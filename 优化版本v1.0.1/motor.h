/*
 * motor.h - TB6612FNG 双电机驱动 (SysConfig 引脚)
 *
 * 左电机: PA12(PWM=TIMG0_CCP0) PA14(IN1) PA15(IN2)
 * 右电机: PA13(PWM=TIMG0_CCP1) PA7(IN1)  PA16(IN2)
 * STBY: 硬件接高
 */

#ifndef MOTOR_H
#define MOTOR_H
#include <stdint.h>

void motor_init(void);
void motor_left_set(int16_t pct);
void motor_right_set(int16_t pct);
void motor_set(int16_t left, int16_t right);
void motor_brake(void);
void motor_coast(void);

#endif
