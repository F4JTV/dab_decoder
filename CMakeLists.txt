cmake_minimum_required(VERSION 3.13)
project(dab_decoder)

# ===========================================================================
# SDR++ DAB / DAB+ Decoder Module
#
# This module embeds welle.io's mature DAB/DAB+ backend (GPL-2 compatible).
# The backend is expected to be available as a git submodule at:
#     decoder_modules/dab_decoder/welle.io/
#
# To populate it inside a fresh SDR++ checkout:
#     cd decoder_modules/dab_decoder
#     git clone --depth 1 https://github.com/AlbrechtL/welle.io.git
#
# (or wire it as a real submodule from SDR++'s root if you fork the repo)
#
# Dependencies:
#   - libfaad2  (HE-AAC v2 decoder for DAB+)
#   - libmpg123 (MP2/MPEG-1 Layer II decoder for DAB legacy)
#   - libfftw3f (OFDM FFT)
#   - libfec    (Reed-Solomon) is vendored inside welle.io/src/libs/fec
#
# Ubuntu 24:   sudo apt install libfaad-dev libmpg123-dev libfftw3-dev
# Windows 11:  vcpkg install faad2 mpg123 fftw3 (and pass CMAKE_TOOLCHAIN_FILE)
# ===========================================================================

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- Module source files (SDR++ side) --------------------------------------
# sdrpp_module.cmake expects ${SRC} to hold the list of source files.
file(GLOB SRC "src/*.cpp" "src/*.c")

# --- welle.io backend location ---------------------------------------------
set(WELLE_IO_DIR "${CMAKE_CURRENT_SOURCE_DIR}/welle.io" CACHE PATH "Path to welle.io source tree")

if(NOT EXISTS "${WELLE_IO_DIR}/src/backend/radio-receiver.h")
    message(FATAL_ERROR
        "welle.io backend not found at ${WELLE_IO_DIR}\n"
        "Please run inside decoder_modules/dab_decoder/:\n"
        "    git clone --depth 1 https://github.com/AlbrechtL/welle.io.git")
endif()

# welle.io backend sources (verbatim list, matching welle.io's own CMakeLists.txt)
set(WELLE_BACKEND_SRC
    "${WELLE_IO_DIR}/src/backend/dab-audio.cpp"
    "${WELLE_IO_DIR}/src/backend/decoder_adapter.cpp"
    "${WELLE_IO_DIR}/src/backend/dab_decoder.cpp"
    "${WELLE_IO_DIR}/src/backend/dabplus_decoder.cpp"
    "${WELLE_IO_DIR}/src/backend/charsets.cpp"
    "${WELLE_IO_DIR}/src/backend/dab-constants.cpp"
    "${WELLE_IO_DIR}/src/backend/mot_manager.cpp"
    "${WELLE_IO_DIR}/src/backend/pad_decoder.cpp"
    "${WELLE_IO_DIR}/src/backend/eep-protection.cpp"
    "${WELLE_IO_DIR}/src/backend/fib-processor.cpp"
    "${WELLE_IO_DIR}/src/backend/fic-handler.cpp"
    "${WELLE_IO_DIR}/src/backend/msc-handler.cpp"
    "${WELLE_IO_DIR}/src/backend/freq-interleaver.cpp"
    "${WELLE_IO_DIR}/src/backend/ofdm-decoder.cpp"
    "${WELLE_IO_DIR}/src/backend/ofdm-processor.cpp"
    "${WELLE_IO_DIR}/src/backend/phasereference.cpp"
    "${WELLE_IO_DIR}/src/backend/phasetable.cpp"
    "${WELLE_IO_DIR}/src/backend/tii-decoder.cpp"
    "${WELLE_IO_DIR}/src/backend/protTables.cpp"
    "${WELLE_IO_DIR}/src/backend/radio-receiver.cpp"
    "${WELLE_IO_DIR}/src/backend/tools.cpp"
    "${WELLE_IO_DIR}/src/backend/uep-protection.cpp"
    "${WELLE_IO_DIR}/src/backend/viterbi.cpp"
)

# welle.io "various" helpers used by the backend
set(WELLE_VARIOUS_SRC
    "${WELLE_IO_DIR}/src/various/Xtan2.cpp"
    "${WELLE_IO_DIR}/src/various/channels.cpp"
    "${WELLE_IO_DIR}/src/various/fft.cpp"
)

# Bundled Reed-Solomon (welle.io/src/libs/fec)
set(WELLE_FEC_SRC
    "${WELLE_IO_DIR}/src/libs/fec/decode_rs_char.c"
    "${WELLE_IO_DIR}/src/libs/fec/encode_rs_char.c"
    "${WELLE_IO_DIR}/src/libs/fec/init_rs_char.c"
)

# --- Glue the whole thing into the SDR++ module ----------------------------
# include(${SDRPP_MODULE_CMAKE}) creates a 'dab_decoder' shared library target
# with the right link/install rules for the SDR++ plugin directory.
include(${SDRPP_MODULE_CMAKE})

# Append the welle.io sources to the target sources
target_sources(dab_decoder PRIVATE
    ${WELLE_BACKEND_SRC}
    ${WELLE_VARIOUS_SRC}
    ${WELLE_FEC_SRC}
)

target_include_directories(dab_decoder PRIVATE
    "src/"
    "${WELLE_IO_DIR}/src/"
    "${WELLE_IO_DIR}/src/backend"
    "${WELLE_IO_DIR}/src/various"
    "${WELLE_IO_DIR}/src/input"
    "${WELLE_IO_DIR}/src/libs/fec"
)

# welle.io's dab-audio.cpp expects either DABLIN_AAC_FAAD2 or DABLIN_AAC_FDKAAC.
# We pick FAAD2 which is what welle.io's own CMake defaults to.
target_compile_definitions(dab_decoder PRIVATE DABLIN_AAC_FAAD2)

# --- Platform-specific dependency wiring -----------------------------------
if(MSVC)
    # Windows 11 — assume vcpkg with the standard triplet
    # vcpkg install faad2 mpg123 fftw3
    # Then pass: -DCMAKE_TOOLCHAIN_FILE=<vcpkg>/scripts/buildsystems/vcpkg.cmake
    find_package(faad CONFIG QUIET)
    if(TARGET faad::faad)
        target_link_libraries(dab_decoder PRIVATE faad::faad)
    else()
        # Fallback: vcpkg installs faad2 as a plain .lib
        find_library(FAAD2_LIB NAMES faad faad2 libfaad libfaad2 REQUIRED)
        find_path(FAAD2_INCLUDE NAMES neaacdec.h REQUIRED)
        target_include_directories(dab_decoder PRIVATE ${FAAD2_INCLUDE})
        target_link_libraries(dab_decoder PRIVATE ${FAAD2_LIB})
    endif()

    find_library(MPG123_LIB NAMES mpg123 libmpg123 mpg123-0 REQUIRED)
    find_path(MPG123_INCLUDE NAMES mpg123.h REQUIRED)
    target_include_directories(dab_decoder PRIVATE ${MPG123_INCLUDE})
    target_link_libraries(dab_decoder PRIVATE ${MPG123_LIB})

    find_package(FFTW3 CONFIG REQUIRED)
    target_link_libraries(dab_decoder PRIVATE FFTW3::fftw3f)

    target_compile_options(dab_decoder PRIVATE /O2 /EHsc /W3 /bigobj)
    # welle.io uses some POSIX-ish constructs; silence the noisier warnings
    target_compile_definitions(dab_decoder PRIVATE
        _CRT_SECURE_NO_WARNINGS
        _USE_MATH_DEFINES
        NOMINMAX
        WIN32_LEAN_AND_MEAN
    )
else()
    # Linux / macOS — pkg-config
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(FAAD2 REQUIRED faad2)
    pkg_check_modules(MPG123 REQUIRED libmpg123)
    pkg_check_modules(FFTW3F REQUIRED fftw3f)

    target_include_directories(dab_decoder PRIVATE
        ${FAAD2_INCLUDE_DIRS}
        ${MPG123_INCLUDE_DIRS}
        ${FFTW3F_INCLUDE_DIRS}
    )
    target_link_directories(dab_decoder PRIVATE
        ${FAAD2_LIBRARY_DIRS}
        ${MPG123_LIBRARY_DIRS}
        ${FFTW3F_LIBRARY_DIRS}
    )
    target_link_libraries(dab_decoder PRIVATE
        ${FAAD2_LIBRARIES}
        ${MPG123_LIBRARIES}
        ${FFTW3F_LIBRARIES}
        pthread
    )

    target_compile_options(dab_decoder PRIVATE
        -O3
        -Wno-unused-parameter
        -Wno-unused-variable
        -Wno-unused-but-set-variable
        -Wno-sign-compare
        -Wno-reorder
        -Wno-deprecated-declarations
    )
endif()
