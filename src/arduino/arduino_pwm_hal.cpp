// SPDX-License-Identifier: MIT
/**
 * @file    arduino_pwm_hal.cpp
 * @brief   Arduino analogWrite → pwm_hal_t 实现（Arduino 头唯一出现处）
 *
 * @note    平台面规模：登记表为静态数组（容量给足常见装配：双定时器、
 *          八路通道），满即如实返回 -1，不静默截断。
 * @note    同频定时器唯一：与核心层约束一致（pwm_timer 同频重复建被拒，
 *          本表以频率为绑定键亦不产生歧义）。
 */

#include <Arduino.h>
#include <math.h>
#include "motor/pwm_hal.h"
#include "arduino_pwm_hal.h"

/* ── 平台面规模（应用常数，非电机参数） ── */
enum { PLAT_TIMERS_MAX = 4, PIN_BINDS_MAX = 8 };

/**
 * @brief   平台定时器记录（timer_config 出参 plat 指向此处的静态槽）
 * @note    同频唯一键：timer_config 拒绝重复频率（与核心层约束一致）。
 */
typedef struct {
    uint32_t freq_hz;    /**< 载波频率，Hz */
    uint8_t  channels;   /**< 通道数（登记，供诊断） */
    int      used;       /**< 占用标记 */
} plat_timer_t;

/**
 * @brief   （载波频率, 通道）→ 引脚 接线登记项
 * @note    last_duty 供 channel_enable(1) 重放（占空单源在平台写点）。
 */
typedef struct {
    uint32_t freq_hz;    /**< 载波频率，Hz */
    uint8_t  channel;    /**< 通道号 */
    int      pin;        /**< Arduino 引脚 */
    float    last_duty;  /**< 最近一次写入占空（0.0~1.0） */
    int      used;       /**< 占用标记 */
} pin_bind_t;

static plat_timer_t s_plat[PLAT_TIMERS_MAX];
static pin_bind_t   s_bind[PIN_BINDS_MAX];

static pin_bind_t *bind_find(uint32_t freq_hz, uint8_t channel)
{
    for (int i = 0; i < PIN_BINDS_MAX; i++)
        if (s_bind[i].used && s_bind[i].freq_hz == freq_hz &&
            s_bind[i].channel == channel)
            return &s_bind[i];
    return NULL;
}

/* ── 输出换算 ────────────────────────────────────────────────
 * ESP32 核心（>=3.x）：analogWriteFrequency 逐引脚设频；分辨率按频段
 *   取档——低频（<=1kHz，舵机 50Hz）12bit（20ms/4096 ≈ 4.9us 步距），
 *   高频 8bit（20kHz 载波时钟预算内）。分辨率与频率须匹配，故在写点
 *   同步设置（analogWriteResolution 逐引脚、幂等）。
 * 其余核心：8bit analogWrite（无逐引脚设频面，仅高频槽正确）。 */
#if defined(ARDUINO_ARCH_ESP32) && defined(ESP_ARDUINO_VERSION_MAJOR) \
    && (ESP_ARDUINO_VERSION_MAJOR >= 3)
#define PWM_GLUE_ESP32_CORE 1
#else
#define PWM_GLUE_ESP32_CORE 0
#endif

/** @brief 输出占空到引脚（0.0~1.0；duty 合法性由 pwm_timer 层单源执法） */
static void write_duty(pin_bind_t *b, float duty)
{
    b->last_duty = duty;
#if PWM_GLUE_ESP32_CORE
    const uint8_t bits = (b->freq_hz <= 1000u) ? 12u : 8u;
    const uint32_t span = (1ul << bits) - 1u;
    analogWriteResolution((uint8_t)b->pin, bits);
    analogWrite((uint8_t)b->pin,
                (int)lroundf(duty * (float)span));
#else
    analogWrite((uint8_t)b->pin, (int)lroundf(duty * 255.0f));
#endif
}

/* ── pwm_hal_t 四成员（契约见核心层 motor/pwm_hal.h） ───────── */

static int pwm_glue_timer_config(uint32_t freq_hz, uint8_t channels,
                                 void **out_plat, void *ctx)
{
    (void)ctx;
    if (out_plat == NULL || freq_hz == 0u || channels == 0u) return -1;
    for (int i = 0; i < PLAT_TIMERS_MAX; i++)
        if (s_plat[i].used && s_plat[i].freq_hz == freq_hz)
            return -1;   /* 同频重复建（核心层约束） */
    for (int i = 0; i < PLAT_TIMERS_MAX; i++) {
        if (!s_plat[i].used) {
            s_plat[i].freq_hz = freq_hz;
            s_plat[i].channels = channels;
            s_plat[i].used = 1;
            *out_plat = &s_plat[i];
            return 0;   /* 平台契约：占空全 0 的安全态（analogWrite 尚未写） */
        }
    }
    return -1;   /* 平台定时器池满 */
}

static int pwm_glue_timer_release(void *plat, void *ctx)
{
    (void)ctx;
    plat_timer_t *t = (plat_timer_t *)plat;
    if (t == NULL || !t->used) return -1;
    /* 先把该频率下全部绑定引脚置安全低电平，再注销登记 */
    for (int i = 0; i < PIN_BINDS_MAX; i++) {
        if (s_bind[i].used && s_bind[i].freq_hz == t->freq_hz) {
            write_duty(&s_bind[i], 0.0f);
            s_bind[i].used = 0;
        }
    }
    t->used = 0;
    return 0;
}

static int pwm_glue_channel_set_duty(void *plat, uint8_t channel,
                                     float duty, void *ctx)
{
    (void)ctx;
    const plat_timer_t *t = (const plat_timer_t *)plat;
    if (t == NULL || !t->used) return -1;
    pin_bind_t *b = bind_find(t->freq_hz, channel);
    if (b == NULL) return -1;   /* 通道未绑定引脚：接线缺失，如实失败 */
    write_duty(b, duty);
    return 0;
}

static int pwm_glue_channel_enable(void *plat, uint8_t channel,
                                   int enable, void *ctx)
{
    (void)ctx;
    const plat_timer_t *t = (const plat_timer_t *)plat;
    if (t == NULL || !t->used) return -1;
    pin_bind_t *b = bind_find(t->freq_hz, channel);
    if (b == NULL) return -1;
    write_duty(b, (enable != 0) ? b->last_duty : 0.0f);
    return 0;
}

/* 平台操作表（顺序初始化：timer_config/timer_release/
 * channel_set_duty/channel_enable/ctx，字段序即 pwm_hal_t 声明序） */
static const pwm_hal_t s_arduino_pwm_hal = {
    pwm_glue_timer_config,
    pwm_glue_timer_release,
    pwm_glue_channel_set_duty,
    pwm_glue_channel_enable,
    NULL,
};

int arduino_pwm_hal_install(void)
{
    return pwm_hal_register(&s_arduino_pwm_hal);
}

int arduino_pwm_hal_bind_pin(pwm_timer_t *t, uint8_t channel, int pin)
{
    if (t == NULL || pin < 0) return -1;
    const uint32_t freq_hz = pwm_timer_freq(t);
    if (freq_hz == 0u) return -1;
    if (bind_find(freq_hz, channel) != NULL) return -1;   /* 重复绑定 */

    pin_bind_t *slot = NULL;
    for (int i = 0; i < PIN_BINDS_MAX; i++) {
        if (!s_bind[i].used) { slot = &s_bind[i]; break; }
    }
    if (slot == NULL) return -1;   /* 登记表满 */

    pinMode((uint8_t)pin, OUTPUT);
#if PWM_GLUE_ESP32_CORE
    analogWriteFrequency((uint8_t)pin, freq_hz);   /* 逐引脚载波 */
#endif
    slot->freq_hz = freq_hz;
    slot->channel = channel;
    slot->pin = pin;
    slot->last_duty = 0.0f;
    slot->used = 1;
    return 0;
}
