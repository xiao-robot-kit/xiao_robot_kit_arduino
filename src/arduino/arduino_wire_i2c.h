// SPDX-License-Identifier: MIT
/**
 * @file    arduino_wire_i2c.h
 * @brief   Arduino Wire(TwoWire) → IMU I2C HAL 函数指针表适配（薄胶水）
 *
 * @note    分层单向：本头只依赖核心层的 imu/imu_hal_i2c.h 契约，不暴露
 *          任何 Arduino 类型（Wire 对象指针经 ctx 挂进操作表）；核心驱动
 *          只认 imu_i2c_bus_t，不 include 任何平台头。
 * @note    返回值语义与核心层统一：0 = 成功；-1 = 失败（无部分成功）。
 * @note    接线一次（init）默认绑 Wire 总线；总线由本胶水独占初始化，
 *          例程侧不要再调 Wire.begin()（重复初始化行为依核心而异）。
 */

#ifndef ARDUINO_WIRE_I2C_H
#define ARDUINO_WIRE_I2C_H

#include <stdint.h>
#include "imu/imu_hal_i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   启动 Wire 总线并把它接进 imu_i2c_bus_t 操作表
 * @param   bus       出参：操作表（调用方持有，栈/静态皆可；非 NULL）
 * @param   speed_hz  总线速率，Hz（如 100000/400000，走核心预设表）
 * @param   sda       SDA 引脚号；-1 = 用核心默认引脚
 * @param   scl       SCL 引脚号；-1 = 用核心默认引脚
 * @return  0 = 成功；-1 = 参数非法
 * @note    显式引脚形态 Wire.begin(sda, scl) 需核心支持（XIAO 系
 *          ESP32/SAMD/nRF52/RP2040 核心均支持；经典 AVR 不支持）。
 */
int arduino_wire_i2c_init(imu_i2c_bus_t *bus, uint32_t speed_hz, int sda, int scl);

#ifdef __cplusplus
}
#endif

#endif /* ARDUINO_WIRE_I2C_H */
