/*
 * stepper.c - D36A步进电机 (GPIO位脉冲, PA30=STEP, PB0=DIR, PA2=EN)
 */
#include "stepper.h"
#include "ti_msp_dl_config.h"

#define STEP_PORT  GPIOA
#define STEP_PIN   DL_GPIO_PIN_30
#define DIR_PORT   GPIOB
#define DIR_PIN    DL_GPIO_PIN_0
#define EN_PORT    GPIOA
#define EN_PIN     DL_GPIO_PIN_2

void stepper_init(void)
{
    DL_GPIO_initDigitalOutput(IOMUX_PINCM5);   /* PA30=STEP */
    DL_GPIO_clearPins(STEP_PORT, STEP_PIN);
    DL_GPIO_enableOutput(STEP_PORT, STEP_PIN);

    DL_GPIO_initDigitalOutput(IOMUX_PINCM12);  /* PB0=DIR */
    DL_GPIO_clearPins(DIR_PORT, DIR_PIN);
    DL_GPIO_enableOutput(DIR_PORT, DIR_PIN);

    DL_GPIO_initDigitalOutput(IOMUX_PINCM7);   /* PA2=EN, 初始低=禁用(高有效) */
    DL_GPIO_clearPins(EN_PORT, EN_PIN);
    DL_GPIO_enableOutput(EN_PORT, EN_PIN);
}

void stepper_enable(void)  { DL_GPIO_setPins(EN_PORT, EN_PIN); }    /* EN=HIGH 使能 */
void stepper_disable(void) { DL_GPIO_clearPins(EN_PORT, EN_PIN); }  /* EN=LOW  禁用 */

void stepper_set_speed(long speed)
{
    if (speed > 0)
        DL_GPIO_setPins(DIR_PORT, DIR_PIN);
    else
        DL_GPIO_clearPins(DIR_PORT, DIR_PIN);
    stepper_enable();
}

void stepper_step(void)
{
    DL_GPIO_setPins(STEP_PORT, STEP_PIN);     /* 上升沿 */
    delay_cycles(100);                         /* ~3us */
    DL_GPIO_clearPins(STEP_PORT, STEP_PIN);   /* 恢复 LOW */
    delay_cycles(100);
}

void stepper_stop(void)
{
    stepper_disable();
    DL_GPIO_clearPins(STEP_PORT, STEP_PIN);
}
