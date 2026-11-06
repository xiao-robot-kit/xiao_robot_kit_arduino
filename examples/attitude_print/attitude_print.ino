// SPDX-License-Identifier: MIT
/**
 * @file    attitude_print.ino
 * @brief   例程：姿态打印（Mahony 互补滤波 → ZYX 欧拉角度数 CSV）
 *
 * @note    链路：LSM6DS3TR-C(g/dps) → mahony_update → 欧拉角(弧度) →
 *          度数 CSV（roll,pitch,yaw → 串口绘图器）。
 * @note    参数零写死：Mahony 增益走本例程应用层预设表（k_att_presets，
 *          追加档位只改表）；dt 恒等于 IMU 预设的 sample_period_ms——
 *          单源无第二 dt 事实（滤波节拍跟随采样节拍）。
 * @note    教学点：ZYX 欧拉角在 pitch ±90° 处万向锁（yaw/roll 融合丢一
 *          自由度）；3D 应用请改用 mahony_get_quat（四元数无奇异）。
 * @note    验收：静止时三轴稳定近 0°；绕 Y 轴缓慢俯仰时 pitch 跟随摆动、
 *          roll/yaw 基本不动（单轴运动不串轴）。
 */

#include <Arduino.h>
#include <string.h>
#include <Wire.h>
#include "imu/lsm6ds3.h"
#include "imu/imu_cfg.h"
#include "attitude/mahony.h"
#include "arduino/arduino_wire_i2c.h"

/* ── 板级接线（换板只改这里；XIAO ESP32S3：SDA=D4/GPIO5，SCL=D5/GPIO6） ── */
enum { PIN_I2C_SDA = 5, PIN_I2C_SCL = 6 };

/* ── 应用常数（串口/重试节奏；非滤波参数） ── */
enum {
    UART_BAUD       = 115200,
    SERIAL_READY_MS = 2000,
    PROBE_RETRY_MS  = 1000,
};

/** 弧度→度（物理换算常数，非可调参数） */
#define ATT_RAD_TO_DEG  57.29577951f

/* ── Mahony 增益预设表（应用层；dt 单源自 IMU 预设，故不入表） ── */
typedef struct {
    const char *name;   /**< 预设名（本例程命名空间 att.print.*） */
    float       kp;     /**< 比例增益：越大收敛越快，过大放大加速度噪声 */
    float       ki;     /**< 积分增益：补偿陀螺零偏漂移（0 = 关闭） */
} att_preset_t;

static const att_preset_t k_att_presets[] = {
    /* 教学档：中等收敛 + 小量积分补零偏（串口波形对照用） */
    { "att.print.teaching", 2.0f, 0.2f },
};

enum { ATT_PRESETS_N = (int)(sizeof(k_att_presets) / sizeof(k_att_presets[0])) };

static const att_preset_t *att_preset_find(const char *name)
{
    if (name == NULL) return NULL;
    for (int i = 0; i < ATT_PRESETS_N; i++)
        if (strcmp(k_att_presets[i].name, name) == 0)
            return &k_att_presets[i];
    return NULL;
}

/* ── 运行态 ── */
static imu_i2c_bus_t s_bus;        /* Wire → 核心驱动操作表（胶水接线） */
static lsm6ds3_t     s_imu;        /* IMU 驱动实例 */
static mahony_t      s_mf;         /* Mahony 滤波器实例 */
static uint16_t      s_period_ms;  /* 采样节拍（IMU 预设源） */
static uint32_t      s_next_ms;    /* 下一拍时刻（millis 域） */
static uint8_t       s_err_cnt = 0;

/** @brief 停机面：致命错误打印后挂起（不静默退出） */
static void fatal(const char *msg)
{
    Serial.println(msg);
    for (;;) delay(PROBE_RETRY_MS);
}

void setup()
{
    Serial.begin(UART_BAUD);
    for (uint32_t i = 0; i < SERIAL_READY_MS / 10u && !Serial; i++)
        delay(10);

    /* 参数源：IMU 档走核心预设表，增益走应用层预设表（零写死） */
    const imu_cfg_t *cfg = imu_cfg_find("imu.attitude.208");
    if (cfg == NULL) fatal("FATAL preset imu.attitude.208 not found");
    const att_preset_t *att = att_preset_find("att.print.teaching");
    if (att == NULL) fatal("FATAL preset att.print.teaching not found");
    s_period_ms = cfg->sample_period_ms;

    if (arduino_wire_i2c_init(&s_bus, cfg->i2c_speed_hz,
                              PIN_I2C_SDA, PIN_I2C_SCL) != 0)
        fatal("FATAL i2c bus init failed");

    lsm6ds3_init(&s_imu, &s_bus);

    for (;;) {
        int r = lsm6ds3_probe(&s_imu);
        if (r == 0) break;
        Serial.println(r == -2 ? "ERR who_am_i mismatch"
                               : "ERR i2c no ack, retry 1s");
        delay(PROBE_RETRY_MS);
    }
    Serial.println("OK probe: LSM6DS3TR-C @0x6A");

    if (lsm6ds3_configure(&s_imu, cfg) != 0)
        fatal("FATAL configure failed");

    /* 滤波器：增益取预设表；dt 取采样节拍（单源，随 IMU 预设走） */
    mahony_init(&s_mf, att->kp, att->ki, (float)s_period_ms / 1000.0f);

    Serial.print("OK attitude: kp ");
    Serial.print(att->kp, 2);
    Serial.print(" ki ");
    Serial.print(att->ki, 2);
    Serial.print(" dt ");
    Serial.print((float)s_period_ms / 1000.0f, 4);
    Serial.println("s");

    Serial.println("roll_deg,pitch_deg,yaw_deg");   /* 绘图器标签行 */
    s_next_ms = millis();
}

void loop()
{
    while ((int32_t)(millis() - s_next_ms) < 0) { /* 等到下一拍 */ }
    s_next_ms += s_period_ms;

    imu_sample_t s;                        /* g / dps */
    if (lsm6ds3_read(&s_imu, &s) != 0) {
        if (++s_err_cnt % 50u == 1u)       /* 限流报错，不淹绘图器 */
            Serial.println("WARN read fail (counter running)");
        return;
    }
    s_err_cnt = 0;

    mahony_update(&s_mf, &s);              /* g/dps 进；加速度无效帧内部跳过 */

    float roll, pitch, yaw;                /* ZYX 约定，弧度 */
    mahony_get_angles(&s_mf, &roll, &pitch, &yaw);

    Serial.print(roll  * ATT_RAD_TO_DEG, 2);  Serial.print(',');
    Serial.print(pitch * ATT_RAD_TO_DEG, 2);  Serial.print(',');
    Serial.println(yaw   * ATT_RAD_TO_DEG, 2);
}
