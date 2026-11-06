// SPDX-License-Identifier: MIT
/**
 * @file    arduino_pwm_hal.h
 * @brief   Arduino analogWrite → PWM HAL 函数指针表适配（薄胶水）
 *
 * @note    分层单向：本头只依赖核心层的 motor/pwm_timer.h 句柄契约，不暴露
 *          任何 Arduino 类型；核心电机库经 pwm_hal_register 吃进本表，
 *          不 include 任何平台头。
 * @note    引脚寻址：电机层 create 校验 cfg.pin_a/pin_b，但平台输出面
 *          按（载波频率, 通道）寻引脚（同频定时器唯一）；因此接线
 *          = bind_pin 登记（定时器句柄 + 通道 + Arduino 引脚）。
 * @note    返回值语义与核心层统一：0 = 成功；-1 = 失败（无部分成功）。
 * @note    覆盖面（如实）：开环 PWM / 直流桥 / 舵机槽；编码器闭环槽
 *          （MOTOR_ENC_CLOSED）需平台正交解码 HAL，本薄胶水不提供。
 */

#ifndef ARDUINO_PWM_HAL_H
#define ARDUINO_PWM_HAL_H

#include <stdint.h>
#include "motor/pwm_timer.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   注册 Arduino PWM 平台表（pwm_hal_t）给 pwm_timer 组件
 * @return  0 = 成功；-1 = 重复注册或表缺员（语义透传 pwm_hal_register）
 * @note    进程级一次，先于任何 pwm_timer_create_preset 调用。
 */
int arduino_pwm_hal_install(void);

/**
 * @brief   绑定（定时器, 通道）→ Arduino 引脚（接线登记）
 * @param   t        PWM 定时器实例（pwm_timer_create_preset 产物；非 NULL）
 * @param   channel  该定时器上的通道号
 * @param   pin      Arduino 引脚号（>= 0；配 pinMode(OUTPUT)）
 * @return  0 = 成功；-1 = 参数非法或（定时器频率, 通道）重复绑定
 * @note    桥类电机（DC/ENC_CLOSED）占 (channel, channel+1) 成对通道，
 *          正/反两通道各绑一个引脚——两次 bind 都须在 motor_create 前
 *          完成；未绑定的通道被写入时按失败返回（接线缺失不静默）。
 * @note    50Hz 舵机档需 ESP32 Arduino 核心 >= 3.x（analogWriteFrequency
 *          支持逐引脚设频）；其余核心仅高频 PWM/桥槽输出正确。
 */
int arduino_pwm_hal_bind_pin(pwm_timer_t *t, uint8_t channel, int pin);

#ifdef __cplusplus
}
#endif

#endif /* ARDUINO_PWM_HAL_H */
