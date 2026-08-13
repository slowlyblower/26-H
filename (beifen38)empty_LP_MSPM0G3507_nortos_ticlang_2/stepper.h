/*
 * stepper.h - D36A步进电机驱动 (GPIO脉冲)
 *
 * PA30=STEP, PB0=DIR, PA2=EN(高有效, D36A)
 */
#ifndef STEPPER_H
#define STEPPER_H

#include <stdint.h>

void stepper_init(void);
void stepper_enable(void);
void stepper_disable(void);
void stepper_set_speed(long speed);  /* 设DIR+EN, 正CW负CCW */
void stepper_step(void);             /* 单步脉冲 */
void stepper_stop(void);

#endif
