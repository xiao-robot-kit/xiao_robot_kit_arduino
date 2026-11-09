<!-- SPDX-License-Identifier: MIT -->
# XIAO Robot Kit Arduino Library

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

Arduino library for the **XIAO Robot Kit** (XIAO ESP32S3): **LSM6DS3TR-C IMU
driver, Mahony attitude fusion, motor control (PWM / servo / H-bridge),
parameter persistence, IMU bias/temperature calibration and heater control**.

⚠️ **Not yet listed in the Arduino Library Manager** — install manually for now
(see Install below); the registry submission has not been filed yet.

Same POSIX-style C core as the
[`xiao_robot_kit_idf`](https://github.com/xiao-robot-kit/xiao_robot_kit_idf)
library — this package adds the thin `Wire` I2C / `analogWrite` PWM glue and
runnable sketches. **Zero hardcoded parameters**: every tunable lives in a
named preset table.

## Install

1. **Manual clone (works today)**: clone this repository into your sketchbook
   libraries folder —
   `~/Documents/Arduino/libraries/xiao_robot_kit_arduino`
   (Windows: `C:\Users\<you>\Documents\Arduino\libraries\...`).
2. ⚠️ Library Manager listing: **pending** — once the submission is accepted,
   searching "**XIAO Robot Kit Libraries**" will install it; until then use the
   manual clone above.
3. Board: **XIAO ESP32S3** (Seeed board package in Boards Manager).

## Quickstart

```cpp
#include <Arduino.h>
#include <Wire.h>
#include "imu/lsm6ds3.h"
#include "imu/imu_cfg.h"
#include "arduino/arduino_wire_i2c.h"   /* src/arduino: Wire → imu_i2c_bus_t glue */

void setup()
{
    Serial.begin(115200);
    const imu_cfg_t *cfg = imu_cfg_find("imu.raw.104");   /* preset table */
    imu_i2c_bus_t bus;
    arduino_wire_i2c_init(&bus, cfg->i2c_speed_hz, /*SDA*/5, /*SCL*/6);
    // ... lsm6ds3_init / lsm6ds3_probe / lsm6ds3_configure ...
}
```

## Sketches

Runnable sketches — all stream CSV for the Arduino IDE Serial Plotter
(115200 baud):

| Sketch | What it shows | Pins used (XIAO ESP32S3) |
| --- | --- | --- |
| [`examples/imu_raw`](examples/imu_raw) | six-axis raw waveform in `g` / `dps` — level board, `az ≈ ±1g` | SDA=D4/GPIO5, SCL=D5/GPIO6 |
| [`examples/motor_sweep`](examples/motor_sweep) | S-curve sweep across three motor kinds: open-loop PWM, SG90 servo, DC H-bridge (+50% ↔ −50%) | D0/GPIO1, D1/GPIO2, bridge pair D8/GPIO7 + D9/GPIO8 |
| [`examples/attitude_print`](examples/attitude_print) | Mahony attitude: `roll,pitch,yaw` in degrees; tilt on one axis stays off the others | SDA=D4/GPIO5, SCL=D5/GPIO6 |

## Glue coverage and limits (`src/arduino/`)

- `arduino_wire_i2c_init()` fills the `imu_i2c_bus_t` function-pointer table
  from `Wire` — the C core never sees an Arduino header.
- `arduino_pwm_hal_install()` + `arduino_pwm_hal_bind_pin()` register an
  `analogWrite`-backed `pwm_hal_t`; bind every timer channel to a pin
  before `motor_create()` (bridge motors occupy two channels).
- Servo slots (50 Hz carrier) need the ESP32 Arduino core >= 3.x
  (per-pin `analogWriteFrequency`); other cores run the high-frequency
  PWM/bridge slots only.
- Encoder-closed-loop motors (`MOTOR_ENC_CLOSED`) need a quadrature-decoder
  HAL — deliberately not part of the thin glue; wire your own `motor_hal_t`.

No Arduino toolchain? The sketches and glue still get a syntax-level check:

```sh
sh testing/arduino_stub/check_syntax.sh
```

## Layout

```
xiao_robot_kit_arduino/
├── library.properties      # Arduino Library Manager metadata
├── LICENSE                 # MIT
├── src/                    # Arduino's only include root — include paths look like
│   ├── imu/                #   #include "imu/lsm6ds3.h", i.e. component directory name
│   ├── attitude/           #   quat + mahony
│   ├── motor/              #   pwm_timer, pid, scurve, motor + preset registry
│   ├── pstore/             #   persistence: registry + framing + CRC32
│   ├── battery/            #   ADC voltage → state of charge
│   ├── calibration/        #   IMU bias + temperature compensation
│   ├── imu_heater/         #   heater state machine (thermal window)
│   └── arduino/            #   thin Arduino glue (Wire I2C, PWM) — hand-written
#
# src/<component>/ files are published copies of the C core (see "Keeping in
# sync" below); only src/arduino/ is authored in this repository.
├── testing/arduino_stub/   #   minimal Arduino.h / Wire.h stubs + check_syntax.sh
└── examples/
    ├── imu_raw/            #   six-axis CSV for the Serial Plotter
    ├── motor_sweep/        #   S-curve sweep, three motor kinds
    └── attitude_print/     #   Mahony roll/pitch/yaw in degrees
```

ESP-IDF users: the same C core ships as an ESP-IDF component in
[`xiao_robot_kit_idf`](https://github.com/xiao-robot-kit/xiao_robot_kit_idf).

## Keeping in sync

`src/<component>/` is generated from the C core sources — do not edit those
files here. The one intended difference is the include prefix: the core library
keeps its component prefix (`#include "att/quat.h"`, `#include "mtr/motor.h"`)
and this Arduino-shaped copy renames it to the directory name
(`#include "attitude/quat.h"`, `#include "motor/motor.h"`), because an Arduino
library has a single include root (`src/`). `src/arduino/` is the only
hand-written part.

## License

[MIT](LICENSE) © XIAO Robot Kit Contributors