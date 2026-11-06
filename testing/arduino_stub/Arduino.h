// SPDX-License-Identifier: MIT
/**
 * @file    Arduino.h
 * @brief   Arduino 核心头最小 stub（仅声明本库例程与胶水用到的 API 面）
 *
 * @note    用途：无 Arduino IDE/核心的环境下做 gcc 语法级自检
 *          （-fsyntax-only）；不是可运行实现。真实核心提供同名 API。
 * @note    覆盖面 = 本库实际调用的最小集合：Serial 打印、时基、
 *          pinMode/analogWrite（含 ESP32 核心 >=3.x 的逐引脚设频族）。
 */

#ifndef ARDUINO_STUB_ARDUINO_H
#define ARDUINO_STUB_ARDUINO_H

#include <stdint.h>
#include <stddef.h>

/* 常量（语义同真实核心） */
#define HEX     16
#define INPUT   0
#define OUTPUT  1
#define LOW     0
#define HIGH    1

#ifdef __cplusplus
extern "C" {
#endif

/* 时基（ms/us 单调时基；回绕语义由调用方差分吸收） */
unsigned long millis(void);
unsigned long micros(void);

/* 阻塞等待（ms/us） */
void delay(unsigned long ms);
void delayMicroseconds(unsigned int us);

/* 数字 IO */
void pinMode(uint8_t pin, uint8_t mode);
void digitalWrite(uint8_t pin, uint8_t val);

/* 模拟输出（8bit 语义；值域由平台核心约定） */
void analogWrite(uint8_t pin, int value);

/* 逐引脚载波/分辨率（ESP32 核心 >=3.x；其他核心无此面） */
void analogWriteFrequency(uint8_t pin, uint32_t freq);
void analogWriteResolution(uint8_t pin, uint8_t bits);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
/** 串口桩（只声明用到的重载面；真实核心为 HardwareSerial） */
class SerialStubClass {
public:
    void begin(unsigned long baud);
    operator bool() const;

    void print(const char *s);
    void print(char c);
    void print(int v);
    void print(unsigned int v);
    void print(unsigned int v, int base);
    void print(float v, int digits);
    void print(unsigned long v, int base);

    void println(void);
    void println(const char *s);
    void println(int v);
    void println(float v, int digits);
    void println(unsigned int v, int base);
    void println(unsigned long v, int base);
};
extern SerialStubClass Serial;
#endif

#endif /* ARDUINO_STUB_ARDUINO_H */
