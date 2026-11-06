// SPDX-License-Identifier: MIT
/**
 * @file    imu_raw.ino
 * @brief   例程：IMU 原始读取波形（六轴 CSV → 串口绘图器滚动）
 *
 * @note    链路：LSM6DS3TR-C(0x6A, Wire I2C) → 驱动换算(g/dps) →
 *          CSV 波形输出（Arduino IDE 串口绘图器 / Serial Plotter）。
 * @note    参数零写死：采样率/量程/总线速率/节拍全部来自核心预设表
 *          imu.raw.104（本例程不出现任何量程/速率魔数）。
 * @note    验收：打开串口绘图器（波特率 115200），六条曲线滚动；
 *          板静止水平放置时 az ≈ +1g（或 -1g，按安装方向），
 *          ax/ay 与三轴陀螺 ≈ 0。
 */

#include <Arduino.h>
#include <Wire.h>
#include "imu/lsm6ds3.h"
#include "imu/imu_cfg.h"
#include "arduino/arduino_wire_i2c.h"

/* ── 板级接线（换板只改这里；XIAO ESP32S3：SDA=D4/GPIO5，SCL=D5/GPIO6） ── */
enum { PIN_I2C_SDA = 5, PIN_I2C_SCL = 6 };

/* ── 应用常数（串口/重试节奏；非传感器参数，不入预设表） ── */
enum {
    UART_BAUD       = 115200,
    SERIAL_READY_MS = 2000,   /* CDC 枚举有限等待 */
    PROBE_RETRY_MS  = 1000,   /* 器件探测重试周期 */
};

/* ── 运行态（setup/loop 共享） ── */
static imu_i2c_bus_t s_bus;          /* Wire → 核心驱动操作表（胶水接线） */
static lsm6ds3_t     s_imu;          /* IMU 驱动实例 */
static uint16_t      s_period_ms;    /* 推流节拍（预设表源） */
static uint32_t      s_next_ms;      /* 下一拍时刻（millis 域） */
static uint8_t       s_err_cnt = 0;  /* 连续读失败计数（限流报错） */

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

    /* 参数源：命名预设表（表注册，零写死） */
    const imu_cfg_t *cfg = imu_cfg_find("imu.raw.104");
    if (cfg == NULL) fatal("FATAL preset imu.raw.104 not found");
    s_period_ms = cfg->sample_period_ms;

    /* I2C 接线：Wire → imu_i2c_bus_t 函数指针表（薄胶水） */
    if (arduino_wire_i2c_init(&s_bus, cfg->i2c_speed_hz,
                              PIN_I2C_SDA, PIN_I2C_SCL) != 0)
        fatal("FATAL i2c bus init failed");

    lsm6ds3_init(&s_imu, &s_bus);

    /* 链路探测：器件不应答则每秒重试（查接线/供电） */
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

    Serial.print("OK stream: ");
    Serial.print((unsigned)cfg->accel_odr_hz);   Serial.print("Hz +-");
    Serial.print((unsigned)cfg->accel_fs_g);     Serial.print("g +-");
    Serial.print((unsigned)cfg->gyro_fs_dps);    Serial.print("dps period ");
    Serial.print((unsigned)cfg->sample_period_ms);
    Serial.println("ms");

    Serial.println("ax,ay,az,gx,gy,gz");   /* 绘图器标签行（g / dps） */
    s_next_ms = millis();
}

void loop()
{
    /* 节拍对齐：millis 域补差等待（无累积漂移；回绕由无符号差吸收） */
    while ((int32_t)(millis() - s_next_ms) < 0) { /* 等到下一拍 */ }
    s_next_ms += s_period_ms;

    imu_sample_t s;                        /* 物理量：g / dps */
    if (lsm6ds3_read(&s_imu, &s) == 0) {
        Serial.print(s.ax, 4);  Serial.print(',');
        Serial.print(s.ay, 4);  Serial.print(',');
        Serial.print(s.az, 4);  Serial.print(',');
        Serial.print(s.gx, 3);  Serial.print(',');
        Serial.print(s.gy, 3);  Serial.print(',');
        Serial.println(s.gz, 3);
    } else {
        if (++s_err_cnt % 50u == 1u)       /* 限流报错，不淹绘图器 */
            Serial.println("WARN read fail (counter running)");
    }
}
