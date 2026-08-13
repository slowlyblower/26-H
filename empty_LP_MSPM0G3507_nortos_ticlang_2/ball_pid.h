/*
 * ball_pid.h - 摆杆平衡球 串级PID (位置PD + 速度P)
 */
#ifndef BALL_PID_H
#define BALL_PID_H

#include <stdint.h>

typedef struct {
    /* 位置环 */
    float pos_kp, pos_ki, pos_kd;
    float pos_integral, pos_last_error;
    float pos_i_limit, pos_target_limit, pos_deadband;
    float target_cm, real_cm;

    /* 速度环 */
    float vel_kp, vel_ki, vel_kd;
    float target_vel, real_vel;
    float vel_integral, vel_last_error;
    float vel_i_limit, vel_out_limit, vel_deadband;

    long output;
    long bias;
} BallPID_t;

extern BallPID_t g_ball_pid;
extern BallPID_t g_ball_pid3;  /* 任务3独立实例 */

void ball_pid_init(void);
void ball_pid_init3(void);
void ball_pid_set_target(float cm);
void ball_pid_set_target3(float cm);
long ball_pid_cascade_run(float pos_cm, float vel_cm_s);
long ball_pid_cascade_run3(float pos_cm, float vel_cm_s);

#endif
