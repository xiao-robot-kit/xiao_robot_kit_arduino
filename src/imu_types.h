// SPDX-License-Identifier: MIT
/**
 * @file    imu_types.h
 * @brief   单根包含路径兼容垫片（Arduino 递归 src/ 构建）
 *
 * @note    背景：核心层按「每组件目录一个 include 根」设计——CMake 形态
 *          下 xiao_attitude 经 target_link_libraries 拿到 src/imu 根，
 *          attitude/mahony.h 直接裸 include "imu_types.h"。
 *          Arduino 库构建只有一个 include 根（src/），裸名解析不到
 *          src/imu/imu_types.h——本垫片把裸名转发到带前缀路径，使
 *          两种构建形态同一棵树共存。
 * @note    仅被单根构建消费；CMake/IDF 形态不经过本文件（其 install
 *          规则按显式文件清单安装，不含本垫片）。
 */

#include "imu/imu_types.h"
