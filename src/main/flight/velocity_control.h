/*
 * velocity_control.h - Custom NED velocity controller for multirotor interceptors
 *
 * 方案B：INAV 仅保留 ANGLE 姿态内环；本模块在 F722 上自建 NED 三轴速度环。
 *
 * 数据流：
 *   RK3588 --(MSP2, NED 三轴速度 cm/s)--> velocityControlSetTarget()
 *   200Hz 周期任务 taskVelocityControl() 运行 PID，缓存 roll/pitch/throttle 输出
 *   每个 PID(1kHz) 周期末尾 velocityControlApplyToRcCommand() 覆盖 rcCommand
 *
 * 坐标约定：
 *   外部指令：NED（X=北 Y=东 Z=向下）
 *   INAV 内部位置估计：NEU（X=北 Y=东 Z=向上），单位 cm / cm/s
 *   垂直轴在入口/反馈处做一次符号转换。
 *
 * This file is part of INAV.
 *
 * INAV is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "common/time.h"

/* 重置控制器：清积分、失能接管。在解锁状态变化/失联时调用 */
void velocityControlReset(void);

/*
 * 由 MSP2 速度指令调用：设置 NED 三轴速度目标。
 *   velN / velE / velD 单位 cm/s；velD 向下为正（NED）。
 * 同时刷新指令时间戳，用作超时判据。
 */
void velocityControlSetTarget(float velN, float velE, float velD);

/* 200Hz 周期任务：读取速度反馈、运行三轴 PID、缓存姿态/油门输出 */
void taskVelocityControl(timeUs_t currentTimeUs);

/*
 * 每个 PID 周期调用（必须在 rcCommand 全部计算完成之后）：
 * 若控制器有效（指令新鲜、已解锁）则覆盖 rcCommand[ROLL/PITCH/THROTTLE]。
 */
void velocityControlApplyToRcCommand(void);

/* 控制器是否正在接管（指令新鲜且已解锁） */
bool velocityControlIsActive(void);
