#!/bin/sh
# Compile the shared core for the ESP32-S3 (FireBeetle 2) with the PlatformIO toolchain.
set -e
cd "$(dirname "$0")/.."
CXX="${CXX_ESP32S3:-$HOME/.platformio/packages/toolchain-xtensa-esp32s3/bin/xtensa-esp32s3-elf-g++}"
mkdir -p build
"$CXX" -std=gnu++17 -O2 -mlongcalls -Wall -Wextra -Wdouble-promotion -Werror \
  -Icore/include -c tests/target/esp32_core_check.cpp -o build/esp32_core_check.o
echo "core compiles for ESP32-S3: build/esp32_core_check.o"
