// dab_input.h — InputInterface for welle.io, fed from an SDR++ VFO stream.
//
// welle.io's OFDM processor calls InputInterface::getSamples() in a pull
// fashion from its own thread. SDR++ delivers IQ samples in a push fashion
// from the DSP thread via dsp::stream<dsp::complex_t>. We bridge the two
// with a lock-free single-producer / single-consumer ring buffer.

#pragma once

#include "backend/radio-controller.h"   // welle.io InputInterface, DSPCOMPLEX
#include <dsp/stream.h>
#include <dsp/types.h>                  // dsp::complex_t (re,im) — same layout as std::complex<float>

#include <atomic>
#include <vector>
#include <cstring>
#include <thread>
#include <chrono>

namespace dab_sdrpp {

// --- Ring buffer ----------------------------------------------------------
// Bounded SPSC ring of std::complex<float>. The producer (SDR++ DSP thread)
// only writes; the consumer (welle.io OFDM thread) only reads. Atomic head/tail
// give us safe access without mutex contention on the audio-rate hot path.
class IQRing {
public:
    explicit IQRing(size_t capacityPow2 = 1u << 20) // 1 Msample = ~512 ms @ 2.048 MHz
        : cap(capacityPow2), mask(capacityPow2 - 1), buf(capacityPow2) {}

    // Producer side. Returns the count actually written (drops oldest if full).
    size_t write(const DSPCOMPLEX* data, size_t count) {
        size_t h = head.load(std::memory_order_relaxed);
        size_t t = tail.load(std::memory_order_acquire);
        size_t free = cap - (h - t);
        if (count > free) {
            // Drop policy: advance the read pointer to drop the oldest samples.
            // For real-time OFDM this is preferable to blocking the DSP thread.
            size_t drop = count - free;
            tail.store(t + drop, std::memory_order_release);
        }
        for (size_t i = 0; i < count; i++) {
            buf[(h + i) & mask] = data[i];
        }
        head.store(h + count, std::memory_order_release);
        return count;
    }

    // Consumer side. Returns count actually read (may be less than 'count').
    size_t read(DSPCOMPLEX* out, size_t count) {
        size_t t = tail.load(std::memory_order_relaxed);
        size_t h = head.load(std::memory_order_acquire);
        size_t available = h - t;
        if (count > available) count = available;
        for (size_t i = 0; i < count; i++) {
            out[i] = buf[(t + i) & mask];
        }
        tail.store(t + count, std::memory_order_release);
        return count;
    }

    size_t available() const {
        return head.load(std::memory_order_acquire) - tail.load(std::memory_order_acquire);
    }

    void clear() {
        tail.store(head.load(std::memory_order_acquire), std::memory_order_release);
    }

private:
    size_t cap;
    size_t mask;
    std::vector<DSPCOMPLEX> buf;
    std::atomic<size_t> head{0};
    std::atomic<size_t> tail{0};
};

// --- InputInterface implementation for welle.io ---------------------------
class SDRppDabInput : public InputInterface {
public:
    SDRppDabInput() = default;

    // Called by the SDR++ DSP thread for each VFO output block.
    // dsp::complex_t shares the same memory layout as std::complex<float>
    // (two consecutive floats: re, im), so a reinterpret_cast is safe here.
    void pushFromVFO(const dsp::complex_t* data, int count) {
        if (!running.load(std::memory_order_acquire)) return;
        ring.write(reinterpret_cast<const DSPCOMPLEX*>(data),
                   static_cast<size_t>(count));
    }

    // ----- InputInterface API (called by welle.io OFDM thread) ------------

    // welle.io expects this to block until 'size' samples are available,
    // or to return early on stop. We poll with a short sleep.
    int32_t getSamples(DSPCOMPLEX* buffer, int32_t size) override {
        int32_t written = 0;
        while (written < size && running.load(std::memory_order_acquire)) {
            size_t got = ring.read(buffer + written, size_t(size - written));
            if (got > 0) {
                written += int32_t(got);
            } else {
                std::this_thread::sleep_for(std::chrono::microseconds(500));
            }
        }
        return written;
    }

    std::vector<DSPCOMPLEX> getSpectrumSamples(int size) override {
        std::vector<DSPCOMPLEX> out(size_t(size), {0.0f, 0.0f});
        ring.read(out.data(), size_t(size));
        return out;
    }

    int32_t getSamplesToRead(void) override {
        return int32_t(ring.available());
    }

    void setFrequency(int frequency) override { freq = frequency; }
    int getFrequency(void) const override { return freq; }
    bool is_ok(void) override { return true; }
    bool restart(void) override {
        ring.clear();
        running.store(true, std::memory_order_release);
        return true;
    }
    void stop(void) override {
        running.store(false, std::memory_order_release);
    }
    void reset(void) override { ring.clear(); }

    float setGain(int /*gain*/) override { return 0.0f; }
    float getGain(void) const override { return 0.0f; }
    int getGainCount(void) override { return 0; }
    void setAgc(bool /*agc*/) override {}
    std::string getDescription(void) override { return "SDR++ VFO bridge"; }

    void shutdown() {
        running.store(false, std::memory_order_release);
        ring.clear();
    }

private:
    IQRing ring;
    std::atomic<bool> running{true};
    int freq = 0;
};

} // namespace dab_sdrpp
