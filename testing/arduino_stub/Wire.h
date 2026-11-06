// SPDX-License-Identifier: MIT
/**
 * @file    Wire.h
 * @brief   Arduino Wire(I2C) 最小 stub（仅声明本库胶水用到的 API 面）
 *
 * @note    用途：无 Arduino 核心环境下 gcc 语法级自检（-fsyntax-only）。
 *          真实核心为 TwoWire（各核心签名略有差异，此处取交集面）。
 */

#ifndef ARDUINO_STUB_WIRE_H
#define ARDUINO_STUB_WIRE_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus

class TwoWire {
public:
    /* 总线启动（带引脚形态需核心支持；XIAO 系核心均有） */
    void begin(void);
    void begin(int sda, int scl);

    /* 总线速率，Hz */
    void setClock(uint32_t hz);

    /* 传输窗口（写相） */
    void beginTransmission(uint8_t addr);
    size_t write(uint8_t v);
    size_t write(const uint8_t *buf, size_t n);
    uint8_t endTransmission(bool sendStop = true);

    /* 读相（返回实际读到的字节数） */
    int requestFrom(int addr, int n);
    int available(void);
    int read(void);
};

extern TwoWire Wire;

#endif /* __cplusplus */

#endif /* ARDUINO_STUB_WIRE_H */
