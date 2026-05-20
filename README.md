# SDR++ DAB / DAB+ Decoder Module

A decoder module for [SDR++](https://github.com/AlexandreRouma/SDRPlusPlus)
that adds **DAB** and **DAB+** reception in Band III (174–240 MHz).

The module wraps the mature [welle.io](https://github.com/AlbrechtL/welle.io)
backend (~10 000 LOC, GPL-2) to handle the full DAB decoding chain — OFDM
synchronization, FIC/FIG, MSC, Viterbi, Reed-Solomon, time deinterleaving,
PRBS descrambling, MPEG-1 Layer II (DAB) and HE-AAC v2 (DAB+) — and exposes
it through a clean SDR++ UI with a 2.048 MHz VFO, service list, audio
routing to the SDR++ sink manager, and dynamic label / signal quality
indicators.

![Alt text](sdrpp_dab_decoder.png)

## Features

- Band III channel selector (5A → 13F) — frequency and bandwidth set
  automatically
- Real-time sync indicator and SNR meter
- Ensemble name and ensemble ID display
- DAB time (decoded from FIG 0/10)
- Service list with codec (DAB / DAB+) and bitrate (kbps)
- Click-to-play, audio routed through the standard SDR++ sink (soundcard,
  network sink, …)
- **Dynamic Label Segment (DLS)** rendering for the currently playing
  service
- Live decoder quality counters: Reed-Solomon corrections, AAC errors,
  frame errors

## Project layout

```
dab_decoder/
├── CMakeLists.txt          # Compiles the module + the embedded welle.io backend
├── README.md
├── welle.io/               # ← to be cloned (git submodule or git clone)
│   └── src/backend/        # 23 .cpp / 30 .h — full DAB decoder
└── src/
    ├── main.cpp            # SDR++ module: VFO, UI, audio routing
    ├── dab_input.h         # Bridge VFO push  →  welle.io pull (lock-free SPSC ring)
    ├── dab_handlers.h      # RadioController + ProgrammeHandler implementations
    └── dab_channels.h      # Band III frequency table
```

## Dependencies

| Component   | Role                              |
|-------------|-----------------------------------|
| `libfaad2`  | HE-AAC v2 decoder (DAB+)          |
| `libmpg123` | MPEG-1 Layer II decoder (DAB)     |
| `libfftw3f` | FFT for OFDM demodulation         |
| `libfec`    | Reed-Solomon (vendored in `welle.io/src/libs/fec`) |

## Building on Ubuntu 24.04

```bash
# 1. System dependencies
sudo apt update
sudo apt install -y build-essential cmake git pkg-config \
    libfaad-dev libmpg123-dev libfftw3-dev \
    libglfw3-dev libvolk-dev libzstd-dev

# 2. Clone SDR++ if needed
git clone https://github.com/AlexandreRouma/SDRPlusPlus.git
cd SDRPlusPlus

# 3. Drop in this module
rm -rf decoder_modules/dab_decoder
tar xzf /path/to/dab_decoder.tar.gz -C decoder_modules/

# 4. Clone the welle.io backend INSIDE the module directory
cd decoder_modules/dab_decoder
git clone --depth 1 https://github.com/AlbrechtL/welle.io.git
cd ../..

# 5. Build
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DOPT_BUILD_DAB_DECODER=ON
make -j$(nproc)

# 6. Install
sudo make install
```

If `OPT_BUILD_DAB_DECODER` is not present in SDR++'s root `CMakeLists.txt`
(it is on recent versions but may differ on older trees), add it manually
before the list of modules:

```cmake
option(OPT_BUILD_DAB_DECODER "Build the DAB/DAB+ decoder module" OFF)
if(OPT_BUILD_DAB_DECODER)
    add_subdirectory("decoder_modules/dab_decoder")
endif()
```

## Usage

1. Launch SDR++
2. In the **Module Manager**, create an instance of `dab_decoder`
3. Pick a Band III channel from the dropdown
4. Wait for acquisition — the **SYNC LOCKED** indicator turns green
5. The service list populates within a few seconds
6. Click any service to start playback

## Technical notes

### Why 2.048 MHz?

DAB uses an OFDM scheme with 1 536 subcarriers spaced 1 kHz apart,
i.e. a 1.536 MHz useful bandwidth. The standard fixes the baseband
sample rate at **2.048 MHz** (= 1.536 × 4/3) for filtering and oversampling.
The module forces the VFO to that rate; the SDR source must support it
(RTL-SDR: yes, Airspy: internal resampling, HackRF: yes).

### Threading

The module orchestrates four threads:

```
┌─────────────────┐   push    ┌──────────────┐  pull   ┌─────────────────┐
│ SDR++ DSP thread│ ────────► │ SPSC ring 1M │ ───────►│ welle.io OFDM   │
│ (iqHandler)     │           │ (lock-free)  │         │ processor thread│
└─────────────────┘           └──────────────┘         └────────┬────────┘
                                                                │
                              ┌──────────────────┐              │
                              │ welle.io decoder │ ◄────────────┘
                              │ thread (DAB/AAC) │
                              └────────┬─────────┘
                                       │ onNewAudio (int16_t)
                                       ▼
                              ┌──────────────────┐
                              │ audioStream      │
                              │ (stereo float)   │
                              └────────┬─────────┘
                                       │
                                       ▼
                              ┌──────────────────┐
                              │ SDR++ SinkManager│
                              │ → audio output   │
                              └──────────────────┘
```

The lock-free ring avoids any mutex on the 2 Msps hot path. The
"drop oldest" overflow policy keeps real-time behaviour rather than
blocking SDR++'s DSP thread.

### Clean shutdown

`audioStream.swap()` can block the welle.io thread if nobody is reading.
On teardown the following order is observed to avoid deadlocks:

1. `audioStream.stopWriter()` — unblocks any pending `swap()`
2. `iqInput->stop()` — unblocks `getSamples()` on the welle.io side
3. `receiver.reset()` — joins welle.io's threads cleanly
4. Re-arm the flags for the next start
5. Tear down the VFO and sink stream

## Known limitations

- **No MOT/Slideshow rendering** — `onMOT()` is wired up but the
  payload is currently dropped. The raw data is available if you want
  to extend the module.
- **No TII visualization** — disabled in `RadioReceiverOptions` to save CPU.
- **One instance maximum** per SDR++ session (`Max instances` is `1`
  in `SDRPP_MOD_INFO`). welle.io is heavy on memory and threads;
  two concurrent DAB decoders saturate a single RTL-SDR in practice.
- **No automatic channel scan** — the channel list is manual. A scan
  mode would be straightforward to add by iterating
  `bandIIIChannels()` and waiting for `synced.load() == true`.

## License

GPL-2 (inherited from the embedded welle.io backend). See
[welle.io/COPYING](https://github.com/AlbrechtL/welle.io/blob/master/COPYING).

## Credits

- **welle.io** by Albrecht Lohofener, Matthias P. Brändli, Jan van Katwijk
  and contributors — complete DAB/DAB+ backend
- **SDR++** — host SDR framework
