/*
 * ball_pid.c - 摆杆平衡球 纯位置PID
 */
#include "ball_pid.h"
#include <math.h>

BallPID_t g_ball_pid;
BallPID_t g_ball_pid3;

void ball_pid_init(void)
{
    g_ball_pid.pos_kp = 0.9f;
    g_ball_pid.pos_ki = 0.01f;
    g_ball_pid.pos_kd = 0.4f;
    g_ball_pid.pos_integral = 0;
    g_ball_pid.pos_last_error = 0;
    g_ball_pid.pos_i_limit = 2.0f;
    g_ball_pid.pos_target_limit = 5.0f;
    g_ball_pid.pos_deadband = 0.1f;
    g_ball_pid.target_cm = 0;
    g_ball_pid.real_cm = 0;

    g_ball_pid.vel_kp = 1.0f;
    g_ball_pid.vel_ki = 0.0f;
    g_ball_pid.vel_kd = 0.0f;
    g_ball_pid.target_vel = 0;
    g_ball_pid.real_vel = 0;
    g_ball_pid.vel_integral = 0;
    g_ball_pid.vel_last_error = 0;
    g_ball_pid.vel_i_limit = 50.0f;
    g_ball_pid.vel_out_limit = 200.0f;
    g_ball_pid.vel_deadband = 0.1f;

    g_ball_pid.output = 0;
    g_ball_pid.bias   = 15; /* CW 15步前馈 */
}

void ball_pid_set_target(float cm)
{
    g_ball_pid.target_cm = cm;
    g_ball_pid.pos_integral = 0;
    g_ball_pid.pos_last_error = 0;
}

long ball_pid_cascade_run(float pos_cm, float vel_cm_s)
{
    /* 位置低通: filt = 0.6×now + 0.4×last */
    static float pos_filt = 0;
    static uint8_t pos_filt_init = 0;
    if (!pos_filt_init) { pos_filt = pos_cm; pos_filt_init = 1; }
    pos_filt = pos_cm * 0.6f + pos_filt * 0.4f;

    float pos_err = g_ball_pid.target_cm - pos_filt;

    float p = pos_err * g_ball_pid.pos_kp;

    g_ball_pid.pos_integral += pos_err * g_ball_pid.pos_ki * 0.001f;
    if (g_ball_pid.pos_integral > g_ball_pid.pos_i_limit)
        g_ball_pid.pos_integral = g_ball_pid.pos_i_limit;
    if (g_ball_pid.pos_integral < -g_ball_pid.pos_i_limit)
        g_ball_pid.pos_integral = -g_ball_pid.pos_i_limit;

    float d = -g_ball_pid.pos_kd * vel_cm_s;

    float out = p + g_ball_pid.pos_integral + d;

    /* 死区 */
    if (fabsf(pos_err) < g_ball_pid.pos_deadband)
        out = 0;

    if (out > g_ball_pid.vel_out_limit) out = g_ball_pid.vel_out_limit;
    if (out < -g_ball_pid.vel_out_limit) out = -g_ball_pid.vel_out_limit;

    g_ball_pid.output = (long)(-out * 200.0f) + g_ball_pid.bias;
    return g_ball_pid.output;
}

/* ========== 任务3独立实例 ========== */

void ball_pid_init3(void)
{
    g_ball_pid3.pos_kp = 0.9f;
    g_ball_pid3.pos_ki = 0.01f;
    g_ball_pid3.pos_kd = 0.40f;
    g_ball_pid3.pos_integral = 0;
    g_ball_pid3.pos_last_error = 0;
    g_ball_pid3.pos_i_limit = 200.0f;
    g_ball_pid3.pos_target_limit = 5.0f;
    g_ball_pid3.pos_deadband = 0.5f;
    g_ball_pid3.target_cm = 0;
    g_ball_pid3.real_cm = 0;

    g_ball_pid3.vel_kp = 1.0f;
    g_ball_pid3.vel_ki = 0.0f;
    g_ball_pid3.vel_kd = 0.0f;
    g_ball_pid3.target_vel = 0;
    g_ball_pid3.real_vel = 0;
    g_ball_pid3.vel_integral = 0;
    g_ball_pid3.vel_last_error = 0;
    g_ball_pid3.vel_i_limit = 50.0f;
    g_ball_pid3.vel_out_limit = 200.0f;
    g_ball_pid3.vel_deadband = 0.1f;

    g_ball_pid3.output = 0;
    g_ball_pid3.bias   = 0;
}

void ball_pid_set_target3(float cm)
{
    g_ball_pid3.target_cm = cm;
    g_ball_pid3.pos_integral = 0;
    g_ball_pid3.pos_last_error = 0;
}

long ball_pid_cascade_run3(float pos_cm, float vel_cm_s)
{
    static float pos_filt = 0;
    static uint8_t pos_filt_init = 0;
    if (!pos_filt_init) { pos_filt = pos_cm; pos_filt_init = 1; }
    pos_filt = pos_cm * 0.6f + pos_filt * 0.4f;

    float pos_err = g_ball_pid3.target_cm - pos_filt;

    float p = pos_err * g_ball_pid3.pos_kp;

    g_ball_pid3.pos_integral += pos_err * g_ball_pid3.pos_ki * 0.001f;
    if (g_ball_pid3.pos_integral > g_ball_pid3.pos_i_limit)
        g_ball_pid3.pos_integral = g_ball_pid3.pos_i_limit;
    if (g_ball_pid3.pos_integral < -g_ball_pid3.pos_i_limit)
        g_ball_pid3.pos_integral = -g_ball_pid3.pos_i_limit;

    float d = -g_ball_pid3.pos_kd * vel_cm_s;

    float out = p + g_ball_pid3.pos_integral + d;

    if (fabsf(pos_err) < g_ball_pid3.pos_deadband) out = 0;
    if (out > g_ball_pid3.vel_out_limit) out = g_ball_pid3.vel_out_limit;
    if (out < -g_ball_pid3.vel_out_limit) out = -g_ball_pid3.vel_out_limit;

    g_ball_pid3.output = (long)(-out * 200.0f) + g_ball_pid3.bias;
    return g_ball_pid3.output;
}
