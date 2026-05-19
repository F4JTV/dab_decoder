// dab_handlers.h — welle.io callback interface implementations.
//
// Two halves:
//   * RadioControllerImpl   : ensemble-level events (sync, SNR, services list, etc.)
//   * ProgrammeHandlerImpl  : per-service audio + dynamic label + slideshow events
//
// All callbacks are invoked from welle.io threads. We store the data behind
// a mutex and a couple of atomics so the SDR++ menu handler (GUI thread)
// can read it safely. The audio side pushes PCM samples into the SDR++
// audio sink stream owned by the module.

#pragma once

#include "backend/radio-controller.h"
#include "backend/dab-constants.h"

#include <dsp/stream.h>
#include <dsp/types.h>

#include <atomic>
#include <mutex>
#include <string>
#include <vector>
#include <deque>
#include <functional>
#include <cstdint>

namespace dab_sdrpp {

// ============================================================================
//  RadioControllerImpl — ensemble-wide events
// ============================================================================
class RadioControllerImpl : public RadioControllerInterface {
public:
    RadioControllerImpl() = default;

    // ---- live state read from the GUI thread -----------------------------
    std::atomic<float> snrDb{0.0f};
    std::atomic<bool>  synced{false};
    std::atomic<bool>  signalPresent{false};
    std::atomic<int>   fineCorr{0};
    std::atomic<int>   coarseCorr{0};
    std::atomic<int>   fibCrcOkCount{0};
    std::atomic<int>   fibCrcFailCount{0};

    // Ensemble name protected by mutex (it's a std::string)
    std::mutex ensembleMtx;
    std::string ensembleName;
    uint16_t    ensembleId = 0;

    // Date/time from FIG 0/10
    std::atomic<int> dabHour{0}, dabMinute{0};

    // ---- callback: new service IDs are reported as they appear -----------
    // The actual service objects must be queried from RadioReceiver afterwards.
    std::function<void(uint32_t)> onServiceAddedCb;

    // ----------------- RadioControllerInterface ---------------------------
    void onSNR(float snr) override {
        snrDb.store(snr, std::memory_order_relaxed);
    }
    void onFrequencyCorrectorChange(int fine, int coarse) override {
        fineCorr.store(fine, std::memory_order_relaxed);
        coarseCorr.store(coarse, std::memory_order_relaxed);
    }
    void onSyncChange(char isSync) override {
        synced.store(isSync != 0, std::memory_order_relaxed);
    }
    void onSignalPresence(bool isSignal) override {
        signalPresent.store(isSignal, std::memory_order_relaxed);
    }
    void onServiceDetected(uint32_t sId) override {
        if (onServiceAddedCb) onServiceAddedCb(sId);
    }
    void onNewEnsemble(uint16_t eId) override {
        std::lock_guard<std::mutex> lck(ensembleMtx);
        ensembleId = eId;
    }
    void onSetEnsembleLabel(DabLabel& label) override {
        std::lock_guard<std::mutex> lck(ensembleMtx);
        ensembleName = label.utf8_label();
    }
    void onDateTimeUpdate(const dab_date_time_t& dt) override {
        dabHour.store(dt.hour, std::memory_order_relaxed);
        dabMinute.store(dt.minutes, std::memory_order_relaxed);
    }
    void onFIBDecodeSuccess(bool crcCheckOk, const uint8_t* /*fib*/) override {
        if (crcCheckOk) fibCrcOkCount.fetch_add(1, std::memory_order_relaxed);
        else            fibCrcFailCount.fetch_add(1, std::memory_order_relaxed);
    }
    void onNewImpulseResponse(std::vector<float>&& /*data*/) override {}
    void onConstellationPoints(std::vector<DSPCOMPLEX>&& /*data*/) override {}
    void onNewNullSymbol(std::vector<DSPCOMPLEX>&& /*data*/) override {}
    void onTIIMeasurement(tii_measurement_t&& /*m*/) override {}
    void onMessage(message_level_t /*level*/,
                   const std::string& /*text*/,
                   const std::string& /*text2*/) override {}
};

// ============================================================================
//  ProgrammeHandlerImpl — per-service audio + metadata
// ============================================================================
//
// welle.io delivers decoded PCM as int16_t interleaved (stereo) at 48 kHz
// (DAB+) or 24/48 kHz (legacy DAB MP2). We convert to stereo float and push
// into the SDR++ audio sink stream provided by the module.
//
// The sink stream is owned by the module (passed in at construction). We
// must not delete it — the module's destructor handles unregistration.
class ProgrammeHandlerImpl : public ProgrammeHandlerInterface {
public:
    ProgrammeHandlerImpl(dsp::stream<dsp::stereo_t>* audioOut)
        : audioStream(audioOut) {}

    // ---- live state ------------------------------------------------------
    std::atomic<int>  currentSampleRate{48000};
    std::atomic<bool> stereo{false};
    std::atomic<int>  frameErrors{0};
    std::atomic<int>  aacErrors{0};
    std::atomic<int>  rsCorrected{0};
    std::atomic<bool> rsUncorrectedErrors{false};
    std::atomic<int>  audioBlocksDelivered{0};

    std::mutex labelMtx;
    std::string dynamicLabel;        // DLS
    std::string audioMode;           // "DAB+", "MP2 stereo", etc.

    // Called from welle.io's decoder thread whenever the audio sample rate
    // changes. The receiver of this callback (the module) is expected to
    // forward the change to the SDR++ sink stream.
    std::function<void(float)> onSampleRateChangeCb;

    // ----------------- ProgrammeHandlerInterface --------------------------
    void onFrameErrors(int errors) override {
        frameErrors.store(errors, std::memory_order_relaxed);
    }

    void onNewAudio(std::vector<int16_t>&& audioData,
                    int sampleRate,
                    const std::string& mode) override
    {
        int prevRate = currentSampleRate.exchange(sampleRate, std::memory_order_acq_rel);
        if (prevRate != sampleRate && onSampleRateChangeCb) {
            onSampleRateChangeCb(float(sampleRate));
        }
        {
            std::lock_guard<std::mutex> lck(labelMtx);
            audioMode = mode;
        }

        // welle.io produces interleaved stereo: L0,R0,L1,R1,...
        // If the source is mono, both channels are filled with the same value.
        const size_t totalSamps = audioData.size();   // total int16_t entries
        if (totalSamps < 2) return;
        const size_t frames = totalSamps / 2;          // L/R frames
        const bool isStereo = true;                    // welle.io always pads to stereo
        stereo.store(isStereo, std::memory_order_relaxed);

        if (!audioStream) return;

        constexpr float k = 1.0f / 32768.0f;
        // Push into the SDR++ stereo stream. We chunk into the writeBuf,
        // calling swap() once per audio packet.
        size_t remaining = frames;
        const int16_t* src = audioData.data();
        while (remaining > 0) {
            int chunk = (remaining > STREAM_BUFFER_SIZE) ? STREAM_BUFFER_SIZE
                                                         : int(remaining);
            for (int i = 0; i < chunk; i++) {
                audioStream->writeBuf[i].l = k * float(src[2*i + 0]);
                audioStream->writeBuf[i].r = k * float(src[2*i + 1]);
            }
            if (!audioStream->swap(chunk)) return;
            src += 2 * chunk;
            remaining -= chunk;
        }
        audioBlocksDelivered.fetch_add(1, std::memory_order_relaxed);
    }

    void onRsErrors(bool uncorrectedErrors, int numCorrectedErrors) override {
        rsUncorrectedErrors.store(uncorrectedErrors, std::memory_order_relaxed);
        rsCorrected.store(numCorrectedErrors, std::memory_order_relaxed);
    }
    void onAacErrors(int errors) override {
        aacErrors.store(errors, std::memory_order_relaxed);
    }
    void onNewDynamicLabel(const std::string& label) override {
        std::lock_guard<std::mutex> lck(labelMtx);
        dynamicLabel = label;
    }
    void onMOT(const mot_file_t& /*mot*/) override {
        // Slideshow (MOT) decoding could go here. We swallow it for now to keep
        // the module focused on audio playback.
    }
    void onPADLengthError(size_t /*announced*/, size_t /*effective*/) override {}

private:
    dsp::stream<dsp::stereo_t>* audioStream;
};

} // namespace dab_sdrpp
