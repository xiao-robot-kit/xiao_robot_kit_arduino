#!/bin/sh
# SPDX-License-Identifier: MIT
#
# Syntax-level check for the Arduino-facing code, no Arduino toolchain
# needed: the sketches (.ino) and the glue (src/arduino/*.cpp) are compiled
# with plain g++ -fsyntax-only against the minimal core stubs in
# testing/arduino_stub/. The include path mirrors the real Arduino library
# build exactly: a single root, src/.
#
# Two passes for the PWM glue:
#   1. generic core  (analogWrite only, 8-bit)
#   2. ESP32 core >= 3.x (analogWriteFrequency / per-pin resolution)
#
# Usage:  sh testing/arduino_stub/check_syntax.sh
# Exit 0 = all units pass.

set -e
cd "$(dirname "$0")/../.."          # repo root

CXX=${CXX:-g++}
STUB="-I testing/arduino_stub"
ROOT="-I src"                        # Arduino-faithful single include root
WARN="-Wall -Wextra"

rc=0
check() {
    desc="$1"; shift
    if "$CXX" -fsyntax-only $WARN $STUB $ROOT "$@"; then
        echo "PASS  $desc"
    else
        echo "FAIL  $desc"
        rc=1
    fi
}

check "glue/arduino_wire_i2c.cpp (generic core)"        src/arduino/arduino_wire_i2c.cpp
check "glue/arduino_pwm_hal.cpp (generic core)"         src/arduino/arduino_pwm_hal.cpp
check "glue/arduino_pwm_hal.cpp (ESP32 core >= 3.x)"    \
      -DARDUINO_ARCH_ESP32 -DESP_ARDUINO_VERSION_MAJOR=3 \
      src/arduino/arduino_pwm_hal.cpp
check "sketch/imu_raw.ino"          -x c++ examples/imu_raw/imu_raw.ino
check "sketch/motor_sweep.ino"      -x c++ examples/motor_sweep/motor_sweep.ino
check "sketch/attitude_print.ino"   -x c++ examples/attitude_print/attitude_print.ino

exit $rc
