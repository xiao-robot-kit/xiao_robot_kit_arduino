// SPDX-License-Identifier: MIT
/**
 * @file    arduino_wire_i2c.cpp
 * @brief   Wire(TwoWire) → imu_i2c_bus_t 实现（Arduino 头唯一出现处）
 */

#include <Arduino.h>
#include <Wire.h>
#include "arduino_wire_i2c.h"

/**
 * @brief   写寄存器：START → addr(W) → reg → data → STOP
 * @note    与核心驱动契约一致：0 = 成功；-1 = 失败。
 * @note    ctx = Wire 对象指针（init 挂入），多器件经 addr 区分。
 */
static int wire_reg_write(uint8_t addr, uint8_t reg,
                          const uint8_t *tx, size_t n, void *ctx)
{
    TwoWire *w = (TwoWire *)ctx;
    if (w == NULL || (n > 0u && tx == NULL)) return -1;
    w->beginTransmission(addr);
    w->write(reg);
    if (n > 0u && w->write(tx, n) != n) {
        (void)w->endTransmission();   /* 失败也收尾，总线不留悬挂态 */
        return -1;
    }
    return (w->endTransmission() == 0u) ? 0 : -1;
}

/**
 * @brief   读寄存器：START → addr(W) → reg → RESTART → addr(R) → n 字节 → STOP
 * @note    endTransmission(false) = repeated start（传感器连读的标准姿势）。
 */
static int wire_reg_read(uint8_t addr, uint8_t reg,
                         uint8_t *rx, size_t n, void *ctx)
{
    TwoWire *w = (TwoWire *)ctx;
    if (w == NULL || (n > 0u && rx == NULL)) return -1;
    w->beginTransmission(addr);
    w->write(reg);
    if (w->endTransmission(false) != 0u) return -1;   /* 地址未应答 */
    if (n == 0u) return 0;
    if (w->requestFrom((int)addr, (int)n) != (int)n) return -1;
    for (size_t i = 0; i < n; i++) {
        if (w->available() == 0) return -1;   /* 短读：按失败处理 */
        rx[i] = (uint8_t)w->read();
    }
    return 0;
}

int arduino_wire_i2c_init(imu_i2c_bus_t *bus, uint32_t speed_hz, int sda, int scl)
{
    if (bus == NULL) return -1;
    if (sda >= 0 && scl >= 0) {
        Wire.begin(sda, scl);
    } else {
        Wire.begin();
    }
    Wire.setClock(speed_hz);
    bus->write = wire_reg_write;
    bus->read  = wire_reg_read;
    bus->ctx   = (void *)&Wire;
    return 0;
}
