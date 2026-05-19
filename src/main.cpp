// main.cpp — SDR++ DAB / DAB+ decoder module
//
// Wires welle.io's mature DAB backend into an SDR++ decoder module:
//   - Creates a 2.048 MHz sample-rate VFO (DAB's native rate) with 1.536 MHz
//     of usable bandwidth
//   - Feeds the IQ stream into a lock-free ring buffer consumed by welle.io's
//     OFDM processor
//   - Listens to welle.io's RadioControllerInterface and ProgrammeHandlerInterface
//     callbacks for ensemble info, service list and PCM audio
//   - Renders the SDR++ side: channel selector (5A..13F), sync/SNR display,
//     service list with point-and-play, and dynamic label scrolling text
//   - Exposes the decoded stereo audio to SDR++'s sink manager so it goes
//     through the regular audio output pipeline
//
// License: GPL-2 (matches the welle.io backend it links to)

#include <imgui.h>
#include <module.h>
#include <core.h>
#include <config.h>
#include <gui/gui.h>
#include <gui/style.h>
#include <gui/tuner.h>
#include <signal_path/signal_path.h>
#include <signal_path/sink.h>
#include <dsp/stream.h>
#include <dsp/types.h>
#include <dsp/sink/handler_sink.h>
#include <utils/flog.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <algorithm>

// welle.io backend
#include "backend/radio-receiver.h"
#include "backend/radio-receiver-options.h"
#include "backend/dab-constants.h"

// Module-local
#include "dab_input.h"
#include "dab_handlers.h"
#include "dab_channels.h"

#define CONCAT(a, b) ((std::string(a) + b).c_str())

SDRPP_MOD_INFO{
    /* Name:            */ "dab_decoder",
    /* Description:     */ "DAB / DAB+ Decoder",
    /* Author:          */ "SDR++ DAB Module (welle.io backend)",
    /* Version:         */ 0, 2, 0,
    /* Max instances    */ 1
};

ConfigManager config;

// Constants -----------------------------------------------------------------
// DAB uses 2.048 MHz baseband sample rate (oversampling of the 1.536 MHz
// signal). We let SDR++ deliver IQ at that rate from the VFO and feed it
// directly into welle.io which expects exactly 2.048 Msps.
constexpr double DAB_INPUT_SAMPLE_RATE = 2048000.0;
constexpr double DAB_VFO_BANDWIDTH     = 1536000.0;
constexpr float  DAB_AUDIO_SAMPLE_RATE = 48000.0f;
constexpr int    SERVICE_POLL_MS       = 500;

// ===========================================================================
//  Module class
// ===========================================================================
class DABDecoderModule : public ModuleManager::Instance {
public:
    DABDecoderModule(std::string nm) {
        name = nm;
        selectedChannelIdx = -1;
        selectedServiceIdx = -1;

        // ---- Load configuration ----
        config.acquire();
        if (config.conf.contains("channel")) {
            std::string ch = config.conf["channel"].get<std::string>();
            const auto& list = dab_sdrpp::bandIIIChannels();
            for (size_t i = 0; i < list.size(); i++) {
                if (ch == list[i].name) { selectedChannelIdx = int(i); break; }
            }
        }
        if (selectedChannelIdx < 0) selectedChannelIdx = 0; // default 5A
        config.release();

        // ---- Build "Channel" combo string (zero-separated, double-null terminated) ----
        rebuildChannelComboText();

        // ---- Audio sink stream registration ----
        // Our ProgrammeHandlerImpl will write decoded PCM into audioStream;
        // the SinkManager::Stream takes it as input and routes it to the
        // currently selected audio output (sound card, network sink, etc.).
        srChangeHandler.ctx = this;
        srChangeHandler.handler = &DABDecoderModule::sinkSampleRateChanged;
        sinkStream.init(&audioStream, &srChangeHandler, DAB_AUDIO_SAMPLE_RATE);
        sigpath::sinkManager.registerStream(name, &sinkStream);

        // ---- Create VFO ----
        // 2.048 MHz baseband, 1.536 MHz bandwidth, snap to 1 Hz (no snap).
        vfo = sigpath::vfoManager.createVFO(name,
            ImGui::WaterfallVFO::REF_CENTER,
            0,                                  // offset from center
            DAB_VFO_BANDWIDTH,                  // bandwidth shown on waterfall
            DAB_INPUT_SAMPLE_RATE,              // output sample rate
            DAB_VFO_BANDWIDTH,                  // min BW
            DAB_VFO_BANDWIDTH,                  // max BW
            true);                              // bandwidth locked
        if (vfo) vfo->setSnapInterval(1);

        // ---- DSP sink that pumps IQ into the welle.io input ----
        iqInput = std::make_unique<dab_sdrpp::SDRppDabInput>();
        ns.init(vfo->output, &DABDecoderModule::iqHandler, this);
        ns.start();

        // ---- welle.io handlers ----
        controller = std::make_unique<dab_sdrpp::RadioControllerImpl>();
        programmeHandler = std::make_unique<dab_sdrpp::ProgrammeHandlerImpl>(&audioStream);

        controller->onServiceAddedCb = [this](uint32_t /*sid*/) {
            // Trigger an immediate poll cycle when a new service is detected.
            serviceListDirty.store(true, std::memory_order_release);
        };

        programmeHandler->onSampleRateChangeCb = [this](float newRate) {
            // welle.io detected a different PCM rate (e.g. 24 kHz mono MP2
            // vs 48 kHz DAB+). Update the SDR++ sink to resample correctly.
            sinkStream.setSampleRate(newRate);
        };

        // ---- Start the audio stream pipeline ----
        sinkStream.start();

        // ---- Start the welle.io receiver ----
        startReceiver();

        // ---- Service list poll thread (decoupled from GUI) ----
        pollRunning.store(true, std::memory_order_release);
        pollThread = std::thread(&DABDecoderModule::pollLoop, this);

        // ---- Register the menu entry ----
        gui::menu.registerEntry(name, menuHandler, this, this);
        enabled = true;

        // ---- Tune to the saved channel ----
        applyChannel(selectedChannelIdx);
    }

    ~DABDecoderModule() {
        gui::menu.removeEntry(name);

        // Stop polling first so it doesn't try to touch a half-destroyed receiver
        pollRunning.store(false, std::memory_order_release);
        if (pollThread.joinable()) pollThread.join();

        // Stop welle.io receiver
        stopReceiver();

        // Stop input sink (no more pushes to the ring)
        ns.stop();
        if (iqInput) iqInput->shutdown();

        // Audio pipeline teardown
        sinkStream.stop();
        sigpath::sinkManager.unregisterStream(name);

        // Delete VFO last so the DSP graph is fully disconnected
        if (vfo) {
            sigpath::vfoManager.deleteVFO(vfo);
            vfo = nullptr;
        }
    }

    void postInit() override {}

    void enable() override {
        if (enabled) return;
        // Recreate the VFO (SDR++ deletes it in disable())
        vfo = sigpath::vfoManager.createVFO(name,
            ImGui::WaterfallVFO::REF_CENTER, lastOffset,
            DAB_VFO_BANDWIDTH, DAB_INPUT_SAMPLE_RATE,
            DAB_VFO_BANDWIDTH, DAB_VFO_BANDWIDTH, true);
        if (vfo) vfo->setSnapInterval(1);
        ns.setInput(vfo->output);
        ns.start();
        if (iqInput) {
            iqInput->reset();
            iqInput->restart();
        }
        startReceiver();
        sinkStream.start();
        enabled = true;
        applyChannel(selectedChannelIdx);
    }

    void disable() override {
        if (!enabled) return;
        stopReceiver();
        ns.stop();
        if (iqInput) iqInput->stop();
        sinkStream.stop();
        if (vfo) {
            lastOffset = vfo->getOffset();
            sigpath::vfoManager.deleteVFO(vfo);
            vfo = nullptr;
        }
        enabled = false;
    }

    bool isEnabled() override { return enabled; }

private:
    // -----------------------------------------------------------------------
    //  Receiver lifecycle
    // -----------------------------------------------------------------------
    void startReceiver() {
        // Fresh state for the new tune
        if (controller) {
            controller->synced.store(false);
            controller->signalPresent.store(false);
            controller->snrDb.store(0.0f);
            std::lock_guard<std::mutex> lck(controller->ensembleMtx);
            controller->ensembleName.clear();
            controller->ensembleId = 0;
        }
        {
            std::lock_guard<std::mutex> lck(servicesMtx);
            services.clear();
        }
        playingServiceId.store(0, std::memory_order_release);

        RadioReceiverOptions rro;
        rro.disableCoarseCorrector = false;
        rro.decodeTII              = false;
        receiver = std::make_unique<RadioReceiver>(*controller, *iqInput, rro);
        receiver->restart(false);
    }

    void stopReceiver() {
        // Important order to avoid deadlocks:
        //   (1) Stop the audio stream's writer side. Any welle.io thread
        //       currently blocked inside ProgrammeHandlerImpl::onNewAudio
        //       on audioStream.swap() will return false immediately.
        //   (2) Stop the IQ input. Any welle.io OFDM thread blocked inside
        //       getSamples() polling our ring will exit its loop.
        //   (3) Destroy the receiver — its destructor joins its threads.
        //   (4) Re-arm everything for the next start.
        audioStream.stopWriter();
        if (iqInput) iqInput->stop();
        receiver.reset();
        audioStream.clearWriteStop();
        if (iqInput) iqInput->restart();
    }

    // -----------------------------------------------------------------------
    //  Channel and service tuning
    // -----------------------------------------------------------------------
    void applyChannel(int idx) {
        const auto& list = dab_sdrpp::bandIIIChannels();
        if (idx < 0 || idx >= (int)list.size()) return;
        selectedChannelIdx = idx;

        // Persist
        config.acquire();
        config.conf["channel"] = list[idx].name;
        config.release(true);

        // Tune the main radio to the channel's center frequency, recentering
        // the VFO at zero offset.
        tuner::tune(tuner::TUNER_MODE_CENTER, name, double(list[idx].frequencyHz));
        if (vfo) vfo->setOffset(0);

        // Reset receiver so we re-scan the new ensemble
        stopReceiver();
        startReceiver();
    }

    void playService(uint32_t serviceId) {
        if (!receiver) return;
        Service s = receiver->getService(serviceId);
        if (s.serviceId == 0) return;
        if (!receiver->serviceHasAudioComponent(s)) return;
        receiver->playSingleProgramme(*programmeHandler, "", s);
        playingServiceId.store(serviceId, std::memory_order_release);
    }

    // -----------------------------------------------------------------------
    //  IQ handler — called by SDR++ DSP thread on every VFO output block
    // -----------------------------------------------------------------------
    static void iqHandler(dsp::complex_t* data, int count, void* ctx) {
        auto* self = (DABDecoderModule*)ctx;
        if (self->iqInput) {
            self->iqInput->pushFromVFO(data, count);
        }
    }

    // -----------------------------------------------------------------------
    //  Sink sample-rate change (from SDR++)
    // -----------------------------------------------------------------------
    static void sinkSampleRateChanged(float /*newRate*/, void* /*ctx*/) {
        // Nothing to do — we emit audio at the rate welle.io provides; SDR++
        // resamples internally before reaching the output device.
    }

    // -----------------------------------------------------------------------
    //  Service list polling
    // -----------------------------------------------------------------------
    struct ServiceEntry {
        uint32_t    sid;
        std::string label;
        std::string codec;     // "DAB", "DAB+", "data"
        int         bitrate;   // kbps
    };

    void pollLoop() {
        while (pollRunning.load(std::memory_order_acquire)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(SERVICE_POLL_MS));
            if (!receiver) continue;
            auto raw = receiver->getServiceList();
            std::vector<ServiceEntry> snapshot;
            snapshot.reserve(raw.size());
            for (const auto& s : raw) {
                ServiceEntry e;
                e.sid = s.serviceId;
                e.label = s.serviceLabel.utf8_label();
                e.codec = "data";
                e.bitrate = 0;
                auto comps = receiver->getComponents(s);
                for (const auto& sc : comps) {
                    if (sc.audioType() == AudioServiceComponentType::DAB) {
                        e.codec = "DAB";
                    } else if (sc.audioType() == AudioServiceComponentType::DABPlus) {
                        e.codec = "DAB+";
                    }
                    auto sub = receiver->getSubchannel(sc);
                    if (sub.subChId != -1) e.bitrate = sub.bitrate();
                }
                snapshot.push_back(std::move(e));
            }
            std::sort(snapshot.begin(), snapshot.end(),
                      [](const ServiceEntry& a, const ServiceEntry& b){
                          return a.label < b.label;
                      });
            {
                std::lock_guard<std::mutex> lck(servicesMtx);
                services = std::move(snapshot);
            }
        }
    }

    // -----------------------------------------------------------------------
    //  Menu handler — runs on the GUI thread
    // -----------------------------------------------------------------------
    static void menuHandler(void* ctx) {
        auto* self = (DABDecoderModule*)ctx;
        float menuWidth = ImGui::GetContentRegionAvail().x;

        if (!self->enabled) { style::beginDisabled(); }

        // ---- Channel selector ------------------------------------------
        ImGui::LeftLabel("Channel");
        ImGui::FillWidth();
        int chIdx = self->selectedChannelIdx;
        if (ImGui::Combo(CONCAT("##dab_chan_", self->name),
                         &chIdx, self->channelComboText.c_str()))
        {
            self->applyChannel(chIdx);
        }

        // ---- Sync + SNR ------------------------------------------------
        bool synced = self->controller && self->controller->synced.load();
        bool sigPresent = self->controller && self->controller->signalPresent.load();
        float snr = self->controller ? self->controller->snrDb.load() : 0.0f;

        ImVec4 syncColor = synced ? ImVec4(0.2f, 0.9f, 0.2f, 1.0f)
                                  : (sigPresent ? ImVec4(0.9f, 0.7f, 0.1f, 1.0f)
                                                : ImVec4(0.6f, 0.6f, 0.6f, 1.0f));
        ImGui::TextColored(syncColor, "%s",
            synced ? "SYNC LOCKED" : (sigPresent ? "Signal detected" : "No signal"));

        char snrBuf[32];
        snprintf(snrBuf, sizeof(snrBuf), "SNR: %.1f dB", snr);
        float snrNorm = std::min(1.0f, std::max(0.0f, (snr + 5.0f) / 30.0f));
        ImGui::ProgressBar(snrNorm, ImVec2(menuWidth, 0), snrBuf);

        // ---- Ensemble label --------------------------------------------
        std::string ensName;
        uint16_t ensId = 0;
        if (self->controller) {
            std::lock_guard<std::mutex> lck(self->controller->ensembleMtx);
            ensName = self->controller->ensembleName;
            ensId   = self->controller->ensembleId;
        }
        if (!ensName.empty()) {
            ImGui::Text("Ensemble: %s", ensName.c_str());
            ImGui::Text("Ensemble ID: 0x%04X", (unsigned)ensId);
        } else {
            ImGui::TextDisabled("Ensemble: ...");
        }

        int hh = self->controller ? self->controller->dabHour.load() : 0;
        int mm = self->controller ? self->controller->dabMinute.load() : 0;
        if (hh != 0 || mm != 0) {
            ImGui::Text("DAB time: %02d:%02d", hh, mm);
        }

        ImGui::Separator();

        // ---- Service list ----------------------------------------------
        ImGui::Text("Services");
        std::vector<ServiceEntry> snap;
        {
            std::lock_guard<std::mutex> lck(self->servicesMtx);
            snap = self->services;
        }
        uint32_t playing = self->playingServiceId.load();

        ImGui::BeginChild(CONCAT("##dab_svc_list_", self->name),
                          ImVec2(menuWidth, 180), true);
        for (size_t i = 0; i < snap.size(); i++) {
            const auto& e = snap[i];
            bool isPlaying = (e.sid == playing);
            char row[256];
            snprintf(row, sizeof(row), "%s %s  [%s %dkbps]##svc_%u",
                     isPlaying ? "\xE2\x96\xB6" : "  ",   // ▶
                     e.label.c_str(),
                     e.codec.c_str(),
                     e.bitrate,
                     e.sid);
            if (ImGui::Selectable(row, isPlaying)) {
                self->playService(e.sid);
            }
        }
        if (snap.empty()) {
            ImGui::TextDisabled("(waiting for ensemble...)");
        }
        ImGui::EndChild();

        // ---- Now playing block -----------------------------------------
        if (playing != 0 && self->programmeHandler) {
            std::string dls;
            std::string mode;
            {
                std::lock_guard<std::mutex> lck(self->programmeHandler->labelMtx);
                dls  = self->programmeHandler->dynamicLabel;
                mode = self->programmeHandler->audioMode;
            }
            int sr = self->programmeHandler->currentSampleRate.load();
            int frameErr = self->programmeHandler->frameErrors.load();
            int aacErr   = self->programmeHandler->aacErrors.load();
            int rsCorr   = self->programmeHandler->rsCorrected.load();
            bool rsBad   = self->programmeHandler->rsUncorrectedErrors.load();

            ImGui::Separator();
            ImGui::Text("Now playing");
            ImGui::Text("Mode: %s  ·  %d Hz", mode.empty() ? "?" : mode.c_str(), sr);
            ImGui::TextColored(rsBad ? ImVec4(0.9f, 0.4f, 0.4f, 1.0f)
                                     : ImVec4(0.7f, 0.7f, 0.7f, 1.0f),
                               "RS corr: %d  ·  AAC err: %d  ·  Frame err: %d",
                               rsCorr, aacErr, frameErr);
            if (!dls.empty()) {
                ImGui::TextWrapped("%s", dls.c_str());
            }
        }

        if (!self->enabled) { style::endDisabled(); }
    }

    // -----------------------------------------------------------------------
    //  Helpers
    // -----------------------------------------------------------------------
    void rebuildChannelComboText() {
        channelComboText.clear();
        for (const auto& c : dab_sdrpp::bandIIIChannels()) {
            channelComboText += c.name;
            channelComboText += '\0';
        }
        channelComboText += '\0';
    }

    // -----------------------------------------------------------------------
    //  Members
    // -----------------------------------------------------------------------
    std::string name;
    bool enabled = false;

    // SDR++ DSP
    VFOManager::VFO* vfo = nullptr;
    dsp::sink::Handler<dsp::complex_t> ns;
    double lastOffset = 0;

    // Audio output (welle.io PCM → SDR++ stereo stream → sink manager)
    dsp::stream<dsp::stereo_t> audioStream;
    SinkManager::Stream sinkStream;
    EventHandler<float> srChangeHandler;

    // welle.io plumbing
    std::unique_ptr<dab_sdrpp::SDRppDabInput>           iqInput;
    std::unique_ptr<dab_sdrpp::RadioControllerImpl>     controller;
    std::unique_ptr<dab_sdrpp::ProgrammeHandlerImpl>    programmeHandler;
    std::unique_ptr<RadioReceiver>                      receiver;

    // Service list shared with GUI
    std::mutex servicesMtx;
    std::vector<ServiceEntry> services;
    std::atomic<bool> serviceListDirty{false};
    std::atomic<uint32_t> playingServiceId{0};

    // Polling thread
    std::thread pollThread;
    std::atomic<bool> pollRunning{false};

    // UI state
    int selectedChannelIdx = 0;
    int selectedServiceIdx = -1;
    std::string channelComboText;
};

// ===========================================================================
//  SDR++ module entry points
// ===========================================================================
MOD_EXPORT void _INIT_() {
    json def = json({});
    def["channel"] = "5A";
    config.setPath(core::args["root"].s() + "/dab_decoder_config.json");
    config.load(def);
    config.enableAutoSave();
}

MOD_EXPORT ModuleManager::Instance* _CREATE_INSTANCE_(std::string name) {
    return new DABDecoderModule(name);
}

MOD_EXPORT void _DELETE_INSTANCE_(void* instance) {
    delete (DABDecoderModule*)instance;
}

MOD_EXPORT void _END_() {
    config.disableAutoSave();
    config.save();
}
