// SPDX-License-Identifier: MIT
/**
 * @file    motor_sweep.ino
 * @brief   例程：电机扫描（三类电机 S 梯度扫掠 + 遥测 CSV）
 *
 * @note    链路：核心电机库（PWM 定时器/通道 + S 梯度）→ Arduino
 *          analogWrite 胶水（pwm_hal_t 函数指针表接线）→ 槽表驱动
 *          三槽扫掠（开环 PWM / SG90 舵机 / 直流 H 桥）→ 串口绘图器。
 * @note    扫描语义：每槽两腿交替目标（如 0.8↔0.2、135°↔45°、
 *          +0.5↔-0.5），腿内七相位 S 梯度（恒定 jerk）整形设定值，
 *          腿间驻留 1s——腿走完（scurve_done）→ 驻留 → retarget 重规划。
 * @note    参数零写死：电机/定时器底版来自核心预设表（motor.pwm.generic
 *          等）；接线与轨迹住本例程槽表（表驱动，改演示只改表）。
 * @note    覆盖面（如实）：编码器闭环槽（MOTOR_ENC_CLOSED）需平台正交
 *          解码 HAL，Arduino 薄胶水未提供——本例程只演示三槽。
 *          50Hz 舵机槽需 ESP32 Arduino 核心 >= 3.x（逐引脚设频）。
 * @note    验收：串口绘图器六线呈 S 形爬升/回落交替；舵机缓慢往复摆动；
 *          H 桥电机正转 1s → 反转 1s 交替（槽2 setpoint 符号翻转）。
 */

#include <Arduino.h>
#include <string.h>
#include "motor/motor.h"
#include "motor/scurve.h"
#include "arduino/arduino_pwm_hal.h"

/* ── 应用常数（节拍/串口；非电机参数） ── */
enum {
    UART_BAUD       = 115200,
    SERIAL_READY_MS = 2000,
    POLL_PERIOD_MS  = 10,      /* 控制环节拍（100Hz） */
    POLL_PERIOD_US  = POLL_PERIOD_MS * 1000,
    DWELL_TICKS     = 100,     /* 腿间驻留 100 拍 = 1s */
    TIMERS_MAX      = 4,       /* 定时器登记容量（同名预设去重后 ≤2） */
};

/* ── 演示槽表（接线 + 轨迹；改引脚/槽数/轨迹只改本表） ──
 * 引脚为 XIAO ESP32S3 GPIO 号：D0=1 D1=2 D8=7 D9=8
 * （D4/D5=I2C，D6/D7=串口，避开）。换板改 pin_a/pin_b。
 * scurve 字段序：vmax[u/s] amax[u/s²] jerk[u/s³] dt[s]；
 * dt 必须等于 POLL_PERIOD_MS/1000（节拍单源）。 */
typedef struct {
    const char   *timer_preset;   /* PWM 定时器预设名（核心预设表） */
    const char   *motor_preset;   /* 电机底版预设名（核心预设表） */
    uint8_t       channel;        /* 定时器上的通道（桥类占 ch 与 ch+1） */
    int           pin_a;          /* 单端/桥正向引脚 */
    int           pin_b;          /* 桥反向引脚（非桥类填 -1 不用） */
    scurve_cfg_t  scurve;         /* S 梯度动力学（域随 kind：duty/deg） */
    float         target[2];      /* 两腿交替目标（duty / deg / 带符号 duty） */
} sweep_slot_t;

static const sweep_slot_t k_slots[] = {
    /* 槽0：开环 PWM（u = duty 0~1）——占空 S 形扫掠，主断言对象 */
    { "pwm.motor.20k", "motor.pwm.generic", 0, 1, -1,
      { 1.2f, 6.0f, 40.0f, 0.01f }, { 0.8f, 0.2f } },
    /* 槽1：SG90 舵机（u = 角度 deg）——135°↔45° 缓慢往复（50Hz 载波） */
    { "pwm.servo.50", "motor.servo.sg90", 0, 2, -1,
      { 360.0f, 1600.0f, 16000.0f, 0.01f }, { 135.0f, 45.0f } },
    /* 槽2：直流 H 桥（u = 带符号 duty）——+50% ↔ −50% 正反转交替 */
    { "pwm.motor.20k", "motor.dc.generic", 1, 7, 8,
      { 1.2f, 6.0f, 40.0f, 0.01f }, { 0.5f, -0.5f } },
};

enum { SLOTS_N = (int)(sizeof(k_slots) / sizeof(k_slots[0])) };

/* ── 定时器登记（同名预设只建一个实例，多槽共享通道面） ── */
static struct { const char *preset; pwm_timer_t *handle; } s_timers[TIMERS_MAX];
static int s_timers_n;

static pwm_timer_t *timer_get(const char *preset)
{
    for (int i = 0; i < s_timers_n; i++)
        if (strcmp(s_timers[i].preset, preset) == 0)
            return s_timers[i].handle;
    if (s_timers_n >= TIMERS_MAX) return NULL;
    pwm_timer_t *t = pwm_timer_create_preset(preset);
    if (t == NULL) return NULL;
    s_timers[s_timers_n].preset = preset;
    s_timers[s_timers_n].handle = t;
    s_timers_n++;
    return t;
}

/* ── 槽运行态 ── */
static motor_t *s_motor[SLOTS_N];
static scurve_t s_sc[SLOTS_N];
static int      s_dwell[SLOTS_N];
static uint8_t  s_tgt[SLOTS_N];          /* 下一腿目标下标（0/1 交替） */
static uint32_t s_last_fault[SLOTS_N];
static uint32_t s_next_us;               /* 下一拍时刻（micros 域） */

/** @brief 停机面：致命错误打印后挂起（不静默退出） */
static void fatal(const char *msg)
{
    Serial.println(msg);
    for (;;) delay(1000);
}

void setup()
{
    Serial.begin(UART_BAUD);
    for (uint32_t i = 0; i < SERIAL_READY_MS / 10u && !Serial; i++)
        delay(10);

    /* PWM 平台表注册（进程级一次，先于任何定时器创建） */
    if (arduino_pwm_hal_install() != 0)
        fatal("FATAL pwm hal install failed");

    for (int i = 0; i < SLOTS_N; i++) {
        const sweep_slot_t *slot = &k_slots[i];

        /* 装配样板三步：建定时器 → 预设底版填接线 → create 电机 */
        pwm_timer_t *t = timer_get(slot->timer_preset);
        if (t == NULL) fatal("FATAL timer preset missing");

        const motor_cfg_t *base = motor_cfg_find(slot->motor_preset);
        if (base == NULL) fatal("FATAL motor preset missing");
        motor_cfg_t cfg = *base;
        cfg.timer   = t;
        cfg.channel = slot->channel;
        cfg.pin_a   = slot->pin_a;
        cfg.pin_b   = slot->pin_b;

        /* 接线登记（桥类占 ch/ch+1 双通道，各绑一引脚） */
        if (arduino_pwm_hal_bind_pin(t, slot->channel, slot->pin_a) != 0)
            fatal("FATAL bind pin_a failed");
        if (slot->pin_b >= 0 &&
            arduino_pwm_hal_bind_pin(t, (uint8_t)(slot->channel + 1u),
                                     slot->pin_b) != 0)
            fatal("FATAL bind pin_b failed");

        s_motor[i] = motor_create(&cfg);
        if (s_motor[i] == NULL) fatal("FATAL motor create failed");

        if (scurve_init(&s_sc[i], &slot->scurve, slot->target[0]) != 0)
            fatal("FATAL scurve init failed");
        s_dwell[i] = DWELL_TICKS;
        s_tgt[i]   = 1;         /* 首腿 target[0]，下一腿切 target[1] */
    }

    Serial.print("OK motor sweep: ");
    Serial.print(SLOTS_N);
    Serial.print(" slots, ");
    Serial.print(1000 / POLL_PERIOD_MS);
    Serial.println("Hz poll");

    Serial.println("slot0_set,slot0_duty,slot1_set,slot1_duty,slot2_set,slot2_duty");
    s_next_us = micros();
}

void loop()
{
    /* 节拍对齐：micros 域补差等待（motor_poll 需单调时基，取拍时刻） */
    while ((int32_t)(micros() - s_next_us) < 0) { /* 等到下一拍 */ }
    s_next_us += POLL_PERIOD_US;

    float setpoint[SLOTS_N];
    float duty[SLOTS_N];

    for (int i = 0; i < SLOTS_N; i++) {
        /* dwell/retarget 状态机：腿走完 → 驻留 N 拍 → 交替目标重规划 */
        if (scurve_done(&s_sc[i])) {
            if (s_dwell[i] > 0) {
                s_dwell[i]--;
            } else {
                if (scurve_retarget(&s_sc[i], k_slots[i].target[s_tgt[i]]) != 0)
                    Serial.println("WARN retarget failed");
                s_tgt[i]   ^= 1u;
                s_dwell[i]  = DWELL_TICKS;
            }
        }

        /* S 梯度推进 → 喂设定值 → 一拍输出 → 取遥测 */
        float sp = scurve_step(&s_sc[i]);
        (void)motor_set_target(s_motor[i], sp);
        (void)motor_poll(s_motor[i], s_next_us);

        motor_state_t st;
        if (motor_get_state(s_motor[i], &st) != 0) {
            st.duty  = 0.0f;
            st.fault = 0xFFFFFFFFu;
        }
        setpoint[i] = sp;
        duty[i]     = st.duty;

        if (st.fault != s_last_fault[i]) {   /* 故障位变化才报（不淹绘图器） */
            s_last_fault[i] = st.fault;
            Serial.print("FAULT slot ");
            Serial.print(i);
            Serial.print(" code 0x");
            Serial.println(st.fault, HEX);
        }
    }

    /* 遥测 CSV：每槽设定值 + 输出占空（槽2 duty 为幅值，符号看 setpoint） */
    for (int i = 0; i < SLOTS_N; i++) {
        if (i > 0) Serial.print(',');
        Serial.print(setpoint[i], 4);
        Serial.print(',');
        Serial.print(duty[i], 4);
    }
    Serial.println();
}
