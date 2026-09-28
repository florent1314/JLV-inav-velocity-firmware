/*
 * velocity_control.c - Custom NED velocity controller for multirotor interceptors
 *
 * 方案B：INAV 仅保留 ANGLE 姿态内环；本模块在 F722 上自建 NED 三轴速度环。
 *
 * 架构：
 *   - 外部指令 NED（Vn,Ve,Vd，cm/s），入口转 NEU（垂直轴取反）
 *   - 水平两轴：速度误差 --PI--> 期望水平加速度(cm/s^2) --投影到机头/右翼-->
 *               atan2 求目标 pitch/roll（完整公式，无小角度假设）
 *   - 垂直轴：速度误差 --PI--> 油门修正(µs)，叠加悬停油门
 *   - 200Hz 计算，结果缓存；每个 PID(1kHz) 周期末尾覆盖 rcCommand
 *
 * 注意：以下 PID 增益为【起始标定值】，必须在台架/试飞中整定，非经验证的最终值。
 *
 * This file is part of INAV.
 *
 * INAV is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "platform.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/filter.h"
#include "common/time.h"

#include "drivers/time.h"

#include "sensors/acceleration.h"
#include "sensors/battery.h"

#include "flight/imu.h"
#include "flight/pid.h"
#include "flight/mixer.h"
#include "flight/velocity_control.h"

#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "navigation/navigation_pos_estimator_private.h"

/* ---------------- 参数（起始值，需整定） ---------------- */

#define VEL_TIMEOUT_US              500000      // 速度指令超时：500ms 无更新则解除接管
#define VEL_FEEDBACK_LPF_HZ         15.0f       // 反馈速度环内低通截止频率
#define VEL_MAX_HORIZ_ACC_CMSS      1500.0f     // 水平加速度输出上限（~1.5g，对应约56°倾角）
#define VEL_MAX_HORIZ_VEL_CMS       12000.0f    // 水平速度目标上限（>90m/s，留余量）
#define VEL_MAX_VERT_VEL_CMS        3000.0f     // 垂直速度目标上限（±30m/s）

// 控制方向符号（台架验证：若某轴反应方向相反，翻转对应符号）
// 默认按航空惯例：roll 正=右滚，pitch 正=抬头
#define VEL_ROLL_SIGN               1.0f
#define VEL_PITCH_SIGN              1.0f

/* ---------------- 内部 PID 结构 ---------------- */

typedef struct {
    float kp;
    float ki;
    float kd;
    float integrator;
    float prevError;
    float iLimit;       // 积分项限幅（输出单位）
    float dLimit;       // 微分项变化率限幅
    float outMin;
    float outMax;
} velPid_t;

static float velPidApply(velPid_t *p, float target, float measurement, float dt)
{
    const float error = target - measurement;

    // P
    const float pTerm = p->kp * error;

    // I（含抗积分饱和限幅）
    if (p->ki > 0.0f && dt > 0.0f) {
        p->integrator += p->ki * error * dt;
        p->integrator = constrainf(p->integrator, -p->iLimit, p->iLimit);
    }

    // D（默认 kd=0；对误差微分并限变化率，抑制噪声）
    float dTerm = 0.0f;
    if (p->kd != 0.0f && dt > 0.0f) {
        float deriv = (error - p->prevError) / dt;
        deriv = constrainf(deriv, -p->dLimit, p->dLimit);
        dTerm = p->kd * deriv;
    }
    p->prevError = error;

    return constrainf(pTerm + p->integrator + dTerm, p->outMin, p->outMax);
}

/* ---------------- 模块状态 ---------------- */

static struct {
    bool        active;
    timeUs_t    lastCmdUs;          // 最近一次速度指令时间戳
    timeUs_t    lastRunUs;          // 最近一次任务执行时间戳（用于 dt）

    // 速度目标（NEU，U 向上，cm/s）
    float       targetN;
    float       targetE;
    float       targetU;

    // 控制器
    velPid_t    pidN;
    velPid_t    pidE;
    velPid_t    pidU;

    // 反馈滤波器
    pt1Filter_t fbN;
    pt1Filter_t fbE;
    pt1Filter_t fbU;

    // 缓存输出（rcCommand 单位）
    int16_t     outRoll;
    int16_t     outPitch;
    int16_t     outThrottle;
} vel = {
    // 水平 N 轴：输出 cm/s^2
    .pidN = { .kp = 3.0f, .ki = 1.5f, .kd = 0.0f, .iLimit = 500.0f, .dLimit = 5000.0f,
              .outMin = -VEL_MAX_HORIZ_ACC_CMSS, .outMax = VEL_MAX_HORIZ_ACC_CMSS },
    // 水平 E 轴
    .pidE = { .kp = 3.0f, .ki = 1.5f, .kd = 0.0f, .iLimit = 500.0f, .dLimit = 5000.0f,
              .outMin = -VEL_MAX_HORIZ_ACC_CMSS, .outMax = VEL_MAX_HORIZ_ACC_CMSS },
    // 垂直 U 轴：输出油门修正 µs（outMin/outMax 在运行时按 hover/idle/max 设置）
    .pidU = { .kp = 1.5f, .ki = 0.8f, .kd = 0.0f, .iLimit = 300.0f, .dLimit = 2000.0f,
              .outMin = -300.0f, .outMax = 300.0f },
};

/* ---------------- 辅助 ---------------- */

static void clearIntegrators(void)
{
    vel.pidN.integrator = 0.0f;
    vel.pidE.integrator = 0.0f;
    vel.pidU.integrator = 0.0f;
    vel.pidN.prevError = 0.0f;
    vel.pidE.prevError = 0.0f;
    vel.pidU.prevError = 0.0f;
}

/* ---------------- 公开 API ---------------- */

void velocityControlReset(void)
{
    vel.active = false;
    clearIntegrators();
}

void velocityControlSetTarget(float velN_cms, float velE_cms, float velD_cms)
{
    // NED -> NEU：垂直轴取反（U = -D）
    vel.targetN = constrainf(velN_cms, -VEL_MAX_HORIZ_VEL_CMS, VEL_MAX_HORIZ_VEL_CMS);
    vel.targetE = constrainf(velE_cms, -VEL_MAX_HORIZ_VEL_CMS, VEL_MAX_HORIZ_VEL_CMS);
    vel.targetU = constrainf(-velD_cms, -VEL_MAX_VERT_VEL_CMS, VEL_MAX_VERT_VEL_CMS);
    vel.lastCmdUs = micros();
}

bool velocityControlIsActive(void)
{
    return vel.active;
}

void taskVelocityControl(timeUs_t currentTimeUs)
{
    // 指令新鲜度 & 解锁状态判据
    const bool fresh = (currentTimeUs - vel.lastCmdUs) < VEL_TIMEOUT_US;
    if (!fresh || !ARMING_FLAG(ARMED)) {
        vel.active = false;
        clearIntegrators();
        vel.lastRunUs = currentTimeUs;
        return;
    }

    // 计算 dt
    float dt = 0.0f;
    if (vel.lastRunUs > 0) {
        dt = (float)(currentTimeUs - vel.lastRunUs) * 1e-6f;
    }
    vel.lastRunUs = currentTimeUs;
    if (dt <= 0.0f || dt > 0.1f) {
        dt = 0.005f;   // 异常时回退到名义 200Hz 周期
    }

    // 首次激活：bumpless transfer，反馈滤波器初始化为当前速度
    if (!vel.active) {
        const float vx = posEstimator.est.vel.v[X];
        const float vy = posEstimator.est.vel.v[Y];
        const float vz = posEstimator.est.vel.v[Z];
        pt1FilterReset(&vel.fbN, vx);
        pt1FilterReset(&vel.fbE, vy);
        pt1FilterReset(&vel.fbU, vz);
        clearIntegrators();
        vel.active = true;
    }

    // 反馈速度（未经过 INAV 3Hz 导航滤波的原始估计值），环内做 15Hz 低通
    const float fbN = pt1FilterApply4(&vel.fbN, posEstimator.est.vel.v[X], VEL_FEEDBACK_LPF_HZ, dt);
    const float fbE = pt1FilterApply4(&vel.fbE, posEstimator.est.vel.v[Y], VEL_FEEDBACK_LPF_HZ, dt);
    const float fbU = pt1FilterApply4(&vel.fbU, posEstimator.est.vel.v[Z], VEL_FEEDBACK_LPF_HZ, dt);

    /* ---- 水平速度环：输出期望加速度（cm/s^2） ---- */
    float aN = velPidApply(&vel.pidN, vel.targetN, fbN, dt);
    float aE = velPidApply(&vel.pidE, vel.targetE, fbE, dt);

    // 限制合成水平加速度幅值
    const float aMag = sqrtf(aN * aN + aE * aE);
    if (aMag > VEL_MAX_HORIZ_ACC_CMSS) {
        const float scale = VEL_MAX_HORIZ_ACC_CMSS / aMag;
        aN *= scale;
        aE *= scale;
    }

    // 投影到当前航向的机头/右翼方向（heading：北0、东正、顺时针）
    const float psi = DECIDEGREES_TO_RADIANS(attitude.raw[YAW]);
    const float cpsi = cosf(psi);
    const float spsi = sinf(psi);
    const float aFwd   = aN * cpsi + aE * spsi;    // 机头前向期望加速度
    const float aRight = -aN * spsi + aE * cpsi;   // 右翼期望加速度

    // 期望姿态（decidegrees）。atan2 完整公式：推力方向(水平 a, 垂直 g)
    // 前向加速需机头下俯 -> pitch 取负；右向加速需右滚 -> roll 取正
    const float pitchTargetDg = VEL_PITCH_SIGN * -RADIANS_TO_DECIDEGREES(atan2f(aFwd, GRAVITY_CMSS));
    const float rollTargetDg  = VEL_ROLL_SIGN  *  RADIANS_TO_DECIDEGREES(atan2f(aRight, GRAVITY_CMSS));

    /* ---- 垂直速度环：输出油门（µs） ---- */
    const float idle  = (float)getThrottleIdleValue();
    const float maxT  = (float)getMaxThrottle();
    const float hover = currentBatteryProfile->nav.mc.hover_throttle;

    // 油门修正范围 = [idle-hover, max-hover]；积分限幅随之
    vel.pidU.outMin = idle - hover;
    vel.pidU.outMax = maxT - hover;
    vel.pidU.iLimit = fminf(fabsf(vel.pidU.outMin), fabsf(vel.pidU.outMax));

    const float thrCorrection = velPidApply(&vel.pidU, vel.targetU, fbU, dt);
    const float throttle = constrainf(hover + thrCorrection, idle, maxT);

    /* ---- 转换为 rcCommand 并缓存 ---- */
    const int16_t maxIncR = (int16_t)pidProfile()->max_angle_inclination[FD_ROLL];
    const int16_t maxIncP = (int16_t)pidProfile()->max_angle_inclination[FD_PITCH];

    vel.outRoll     = pidAngleToRcCommand(rollTargetDg, maxIncR);
    vel.outPitch    = pidAngleToRcCommand(pitchTargetDg, maxIncP);
    vel.outThrottle = (int16_t)throttle;
}

void velocityControlApplyToRcCommand(void)
{
    if (vel.active) {
        rcCommand[ROLL]     = vel.outRoll;
        rcCommand[PITCH]    = vel.outPitch;
        rcCommand[THROTTLE] = vel.outThrottle;
    }
}
