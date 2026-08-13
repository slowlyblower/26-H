/*
 * H题「自动行驶小车」
 *
 * 控制逻辑: 8路灰度 PD 循迹(参考 MSPM0G3507_for_car)
 */

#include "ti_msp_dl_config.h"
#include "sensor.h"
#include "motor.h"
#include "encoder.h"
#include "pid.h"
#include "lcd.h"
#include "vision.h"
#include "ball_pid.h"
#include "stepper.h"

/* ================================================================
 *  速度参数 (直接 PWM 占空比 0~100)
 * ================================================================ */
#define SPEED_LINE    38      /* 循迹速度 */

/* ================================================================
 *  全局状态
 * ================================================================ */
static int16_t  g_last_L    = 0;       /* 上帧左轮 PWM */
static int16_t  g_last_R    = 0;       /* 上帧右轮 PWM */

/* ================================================================
 *  IO 引脚
 * ================================================================ */
#define LED_G_PORT   GPIOB
#define LED_G_PIN    DL_GPIO_PIN_24
#define LED_G_IOMUX  IOMUX_PINCM52
#define LED_B_PORT   GPIOB
#define LED_B_PIN    DL_GPIO_PIN_25
#define LED_B_IOMUX  IOMUX_PINCM56  /* PB25, 修复: 原PINCM27是PB10 */
#define KEY1_PORT    GPIOB
#define KEY1_PIN     DL_GPIO_PIN_21
#define KEY1_IOMUX   IOMUX_PINCM49
#define KEY2_PORT    GPIOB
#define KEY2_PIN     DL_GPIO_PIN_20
#define KEY2_IOMUX   IOMUX_PINCM48

/* ---- 基础函数 ---- */
void delay_ms(uint32_t ms) { delay_cycles(ms * 32000); }

static void beep(uint16_t ms)
{
    DL_GPIO_setPins(LED_B_PORT, LED_B_PIN);
    delay_ms(ms);
    DL_GPIO_clearPins(LED_B_PORT, LED_B_PIN);
}

static void extra_io_init(void)
{
    DL_GPIO_initDigitalOutput(LED_G_IOMUX);
    DL_GPIO_clearPins(LED_G_PORT, LED_G_PIN);
    DL_GPIO_enableOutput(LED_G_PORT, LED_G_PIN);
    DL_GPIO_initDigitalOutput(LED_B_IOMUX);
    DL_GPIO_clearPins(LED_B_PORT, LED_B_PIN);
    DL_GPIO_enableOutput(LED_B_PORT, LED_B_PIN);
    DL_GPIO_initDigitalInputFeatures(KEY1_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(KEY2_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
}

static uint8_t key1_read(void) { return DL_GPIO_readPins(KEY1_PORT, KEY1_PIN) ? 1 : 0; }
static uint8_t key2_read(void) { return DL_GPIO_readPins(KEY2_PORT, KEY2_PIN) ? 1 : 0; }

/* ---- 任务选择 ---- */
static uint8_t task_select(void)
{
    uint8_t task = 1;
    lcd_clear(LCD_BLACK);
    lcd_set_cursor_big(0, 0); lcd_puts_big("Select Task:");
    lcd_set_cursor_big(1, 0); lcd_puts_big("K1:+ K2:OK  T");
    lcd_putc_big('0' + task);
    while (1) {
        if (key1_read() == 0) { delay_ms(50);
            if (key1_read() == 0) {
                task = (task % 10) + 1;
                lcd_set_cursor_big(1, 0);
                lcd_puts_big("K1:+ K2:OK  T");
                lcd_putc_big('0' + task);
                lcd_puts_big("  ");
                while (key1_read() == 0) delay_ms(10);
            }
        }
        if (key2_read() == 0) { delay_ms(50);
            if (key2_read() == 0) {
                while (key2_read() == 0) delay_ms(10);
                break;
            }
        }
        delay_ms(50);
    }
    lcd_clear(LCD_BLACK);
    return task;
}

/* ================================================================
 *  循迹控制: 加权位置 + PID (参考 8 路灰度模块源码)
 *
 *  丢线时 (pos == -128): 保持上帧误差, PID 状态连续, 不跳变
 *  有线时: 加权位置 → 死区 → 过零积分清零 → 动态积分限幅 → PID
 * ================================================================ */

void line_follow_reset(void)
{
    g_last_L = 0;
    g_last_R = 0;
}

/*
 * 灰度 + 位置低通滤波 + PD 循迹 (参考 MSPM0G3507_for_car)
 *
 * real = pos * 0.6 + last_pos * 0.4   // 低通滤波
 * out  = error * Kp + delta_error * Kd  // PD, 无积分
 */
#define RIF_TARGET  0.0f    /* 目标位置 (0=居中) */
#define RIF_KP      0.3f    /* P 增益 */
#define RIF_KD      1.2f    /* D 增益 (大KD防抖) */

void line_follow(void)
{
    int8_t pos = sensor_get_position();

    static float last_pos = 0, last_error = 0;
    float steer;

    if (pos == -128) {
        steer = 0;              /* 无线: 直走 */
        last_error = 0;         /* 重置误差历史 */
    } else {
        /* 低通滤波 */
        float filt = (float)pos * 0.6f + last_pos * 0.4f;
        last_pos = filt;

        /* PD */
        float error = RIF_TARGET - filt;
        steer = error * RIF_KP + (error - last_error) * RIF_KD;
        last_error = error;
    }

    if (steer >  40.0f) steer =  40.0f;
    if (steer < -40.0f) steer = -40.0f;

    int16_t L = (int16_t)((float)SPEED_LINE + steer) + 1;
    int16_t R = (int16_t)((float)SPEED_LINE - steer);

    if (L < 0) L = 0; if (L > 100) L = 100;
    if (R < 0) R = 0; if (R > 100) R = 100;

    g_last_L = L;
    g_last_R = R;
    motor_set(L, R);
}

/*
 * 循迹控制 (带速度曲线): 灰度 PD + 低通, speed=基础占空比
 * 任务4/5/6 共用, 替代各自内联的循迹代码
 */
void line_follow_speed(uint16_t speed)
{
    if (sensor_is_line_detected()) {
        int8_t pos = sensor_get_position();
        static float last = 0, le = 0;
        if (pos == -128) {
            le = 0;
            motor_set(speed, speed);
        } else {
            float f = (float)pos * 0.6f + last * 0.4f; last = f;
            float e = 0 - f;
            float s = e * 0.3f + (e - le) * 1.2f; le = e;
            if (s > 40) s = 40; if (s < -40) s = -40;
            float curve = (float)speed * (1.0f - (s>0?s:-s)/40.0f * 0.3f);
            int16_t L = (int16_t)(curve + s);
            int16_t R = (int16_t)(curve - s);
            if (L < 0) L = 0; if (L > 100) L = 100;
            if (R < 0) R = 0; if (R > 100) R = 100;
            motor_set(L, R);
        }
    } else {
        motor_set(speed, speed);
    }
}

/* ================================================================
 *  main
 * ================================================================ */

int main(void)
{
    /* ---- 初始化 ---- */
    SYSCFG_DL_init();
    extra_io_init();

    /* PB10: 外设电源使能 */
    {
        DL_GPIO_initDigitalOutput(IOMUX_PINCM27);
        DL_GPIO_clearPins(GPIOB, DL_GPIO_PIN_10);
        DL_GPIO_enableOutput(GPIOB, DL_GPIO_PIN_10);
    }
    delay_ms(100);
    DL_GPIO_setPins(GPIOB, DL_GPIO_PIN_10);
    delay_ms(150);

    lcd_init();

    /* 任务选择 */
    uint8_t task = task_select();

    /* 驱动初始化 */
    motor_init();
    sensor_init();
    encoder_init();

    pid_init(&g_pid_line, 0.6f, 0.0f, 0.0f, -35.0f, 35.0f);  /* 循迹 PD */
    motor_coast();

    lcd_clear(LCD_BLACK);
    lcd_set_cursor_big(0, 0); lcd_puts_big("Go!");

    /* 传感器预稳定: 读 500ms 让 LM393 上电稳定 */
    uint8_t dummy[8];
    for (int i = 0; i < 50; i++) {
        sensor_read_all(dummy);
        delay_ms(10);
    }

    /* ================================================================
     *  任务5: 赛题要求5 — 循迹+球稳定在中心, ≤30s, 误差≤1cm
     * ================================================================ */
    if (task == 5) {
        #define T5_CENTER_X    319.0f
        #define T5_PIX_PER_CM  28.7f
        #define T5_BALL_CM()   ((T5_CENTER_X - g_ball_x) / T5_PIX_PER_CM)

        vision_init();
        ball_pid_init();
        g_ball_pid.bias = 0;
        stepper_init();
        encoder_reset_distance();

        ball_pid_set_target(-0.1f);
        delay_ms(200);
        stepper_enable();

        /* PB1=TX 拉高, PB3=RX 下拉等待 */
        DL_GPIO_initDigitalOutput(IOMUX_PINCM13);
        DL_GPIO_setPins(GPIOB, DL_GPIO_PIN_1);
        DL_GPIO_enableOutput(GPIOB, DL_GPIO_PIN_1);
        DL_GPIO_initDigitalInputFeatures(IOMUX_PINCM16,
            DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_DOWN,
            DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

        lcd_set_cursor_big(0, 0); lcd_puts_big("T5 Loop");

        uint8_t  _f5[8];
        uint16_t t5 = 0;
        uint32_t t5_prev_ms = 0;
        float t5_vel = 0, t5_prev_cm = 0, t5_last_cm = 0;
        int32_t t5_step_pos = 0;
        float    t5_speed_f = 10.0f;
        uint16_t t5_elapsed = 0;

        /* RX触发: 等1s→CCW20步(2s) */
        static uint8_t t5_rx_flag = 0;
        uint8_t t5_rx_step = 0;
        while (1) {
            t5++;

            /* RX: PB3高→等1s→CCW20步 */
            {static uint8_t ph=0;static uint16_t c=0,s=0;
            if(!t5_rx_flag&&DL_GPIO_readPins(GPIOB,DL_GPIO_PIN_3)){t5_rx_flag=1;ph=1;c=0;}
            if(ph==1){c++;if(c>=500){ph=2;c=0;s=0;}}
            if(ph==2){c++;if(c>=100){t5_rx_step=1;s++;c=0;if(s>=20)ph=3;}}}

            /* 小车加速: 初速10, 每秒+4, 30封顶 */
            if (t5_speed_f < 30.0f && t5 % 500 == 0) t5_speed_f += 4.0f;
            if (t5_speed_f > 30.0f) t5_speed_f = 30.0f;
            uint16_t t5_speed = (uint16_t)t5_speed_f;

            sensor_read_all(_f5);
            encoder_update();
            float d5 = encoder_get_distance();

            /* 330~350cm: 屏蔽左3路+右1路 */
            if (d5 > 330.0f && d5 < 350.0f)
                sensor_set_mask(0x78);
            else
                sensor_set_mask(0xFF);

            /* 球平衡 */
            float b5_cm = T5_BALL_CM();
            if (!g_ball_valid) b5_cm = t5_last_cm; else t5_last_cm = b5_cm;
            if (g_ball_valid && t5 - t5_prev_ms >= 50) {
                float dt = (float)(t5 - t5_prev_ms) / 1000.0f;
                if (dt > 0) t5_vel = (b5_cm - t5_prev_cm) / dt;
                t5_prev_cm = b5_cm; t5_prev_ms = t5;
            }

            long a5 = ball_pid_cascade_run(b5_cm, t5_vel);

            /* 步数限位 */
            float t5_err_abs = (b5_cm > 0) ? b5_cm : -b5_cm;
            int32_t t5_lim = (int32_t)(t5_err_abs * 50);
            if (t5_lim < 0)  t5_lim = 0;
            if (t5_lim > 70) t5_lim = 70;
            if (t5_step_pos >=  t5_lim && a5 > 0) a5 = 0;
            if (t5_step_pos <= -t5_lim && a5 < 0) a5 = 0;

            if (t5_rx_step) {
                t5_rx_step = 0;
                stepper_set_speed(-1);
                stepper_step();
                delay_cycles(64000);
            } else if (a5 != 0) {
                stepper_set_speed(a5);
                stepper_step();
                t5_step_pos += (a5 > 0) ? 1 : -1;
                delay_cycles(64000);
            } else {
                delay_ms(1);
            }

            /* 循迹(每10次更新) */
            if (t5 % 10 == 0) line_follow_speed(t5_speed);

            /* LCD (每200ms) */
            t5_elapsed++;
            if (t5_elapsed % 200 == 0) {
                lcd_set_cursor_big(0, 0);
                lcd_puts_big("T"); lcd_print_int_big((int32_t)(t5_elapsed / 1000));
                lcd_puts_big("s P"); lcd_print_int_big((int32_t)(b5_cm * 10));
                lcd_set_cursor_big(1, 0);
                lcd_puts_big("S"); lcd_print_int_big(t5_step_pos);
                lcd_puts_big(" V"); lcd_print_int_big(t5_speed);
            }
        }
    }

    /* ================================================================
     *  任务6: 赛题要求6 — 循迹+球稳定在任意指定位置, ≤30s, 误差≤1cm
     *
     *  调试: K1→目标+1cm, K2→目标+0.1cm, ±12.5折返
     *  行驶: 同任务5, 球PID跟踪锁定位置
     * ================================================================ */
    if (task == 6) {
        #define T6_CENTER_X    319.0f
        #define T6_PIX_PER_CM  28.7f
        #define T6_BALL_CM()   ((T6_CENTER_X - g_ball_x) / T6_PIX_PER_CM)

        vision_init();
        ball_pid_init();
        g_ball_pid.bias = 0;
        g_ball_pid.pos_deadband = 0.2f;
        stepper_init();
        encoder_reset_distance();

        /* ---- 调试模式: K1+1cm K2+0.1cm, ±12.5折返 ---- */
        float t6_target = 0;
        lcd_set_cursor_big(0, 0); lcd_puts_big("T6 Set Tgt");
        while (1) {
            lcd_set_cursor_big(1, 0);
            lcd_puts_big("T:"); lcd_print_int_big((int32_t)(t6_target * 10));
            lcd_puts_big("mm  ");
            if (key1_read() == 0) {
                delay_ms(30);
                if (key1_read() == 0) {
                    t6_target += 1.0f;
                    if (t6_target > 12.5f) t6_target = -12.5f;
                    while (key1_read() == 0) delay_ms(10);
                }
            }
            /* K2: 短按+0.1, 长按2s确认 */
            if (key2_read() == 0) {
                delay_ms(30);
                if (key2_read() == 0) {
                    uint16_t hold = 0;
                    while (key2_read() == 0) { delay_ms(10); hold += 10; }
                    if (hold >= 2000) break;  /* 长按: 确认 */
                    t6_target += 0.1f;        /* 短按: +0.1 */
                    if (t6_target > 12.5f) t6_target = -12.5f;
                }
            }
            delay_ms(50);
        }

        ball_pid_set_target(t6_target);
        delay_ms(200);
        stepper_enable();

        /* PB1=TX 拉高, PB3=RX 下拉等待 */
        DL_GPIO_initDigitalOutput(IOMUX_PINCM13);
        DL_GPIO_setPins(GPIOB, DL_GPIO_PIN_1);
        DL_GPIO_enableOutput(GPIOB, DL_GPIO_PIN_1);
        DL_GPIO_initDigitalInputFeatures(IOMUX_PINCM16,
            DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_DOWN,
            DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

        lcd_set_cursor_big(0, 0); lcd_puts_big("T6 Loop");
        lcd_set_cursor_big(1, 0);
        lcd_puts_big("T:"); lcd_print_int_big((int32_t)(t6_target * 10));
        lcd_puts_big("mm");

        uint8_t  _f6[8];
        uint16_t t6 = 0;
        uint32_t t6_prev_ms = 0;
        float t6_vel = 0, t6_prev_cm = 0, t6_last_cm = 0;
        int32_t t6_step_pos = 0;
        float    t6_speed_f = 10.0f;
        uint16_t t6_elapsed = 0;

        /* RX触发: 等1s→CCW20步(2s) */
        static uint8_t t6_rx_flag = 0;
        uint8_t t6_rx_step = 0;
        while (1) {
            t6++;

            /* RX: PB3高→等1s→CCW20步 */
            {static uint8_t ph=0;static uint16_t c=0,s=0;
            if(!t6_rx_flag&&DL_GPIO_readPins(GPIOB,DL_GPIO_PIN_3)){t6_rx_flag=1;ph=1;c=0;}
            if(ph==1){c++;if(c>=500){ph=2;c=0;s=0;}}
            if(ph==2){c++;if(c>=100){t6_rx_step=1;s++;c=0;if(s>=20)ph=3;}}}

            /* 小车加速: 初速10, 每秒+4, 30封顶 */
            if (t6_speed_f < 30.0f && t6 % 500 == 0) t6_speed_f += 4.0f;
            if (t6_speed_f > 30.0f) t6_speed_f = 30.0f;
            uint16_t t6_speed = (uint16_t)t6_speed_f;

            sensor_read_all(_f6);
            encoder_update();
            float d6 = encoder_get_distance();

            /* 330~350cm: 屏蔽左3路+右1路 */
            if (d6 > 330.0f && d6 < 350.0f)
                sensor_set_mask(0x78);
            else
                sensor_set_mask(0xFF);

            /* 球平衡 */
            float b6_cm = T6_BALL_CM();
            if (!g_ball_valid) b6_cm = t6_last_cm; else t6_last_cm = b6_cm;
            if (g_ball_valid && t6 - t6_prev_ms >= 50) {
                float dt = (float)(t6 - t6_prev_ms) / 1000.0f;
                if (dt > 0) t6_vel = (b6_cm - t6_prev_cm) / dt;
                t6_prev_cm = b6_cm; t6_prev_ms = t6;
            }

            long a6 = ball_pid_cascade_run(b6_cm, t6_vel);

            /* 步数限位 (同任务5: |e|×50, 上限70) */
            float t6_err_abs = fabsf(b6_cm - t6_target);
            int32_t t6_lim = (int32_t)(t6_err_abs * 50);
            if (t6_lim < 0)  t6_lim = 0;
            if (t6_lim > 70) t6_lim = 70;

            if (t6_step_pos >=  t6_lim && a6 > 0) a6 = 0;
            if (t6_step_pos <= -t6_lim && a6 < 0) a6 = 0;

            /* 死区内归零 */
            if (a6 == 0 && t6_step_pos != 0) {
                a6 = (t6_step_pos > 0) ? -1 : 1;
            }

            if (t6_rx_step) {
                t6_rx_step = 0;
                stepper_set_speed(-1);
                stepper_step();
                delay_cycles(64000);
            } else if (a6 != 0) {
                stepper_set_speed(a6);
                stepper_step();
                t6_step_pos += (a6 > 0) ? 1 : -1;
                delay_cycles(64000);
            } else {
                delay_ms(1);
            }

            /* 循迹(每10次更新) */
            if (t6 % 10 == 0) line_follow_speed(t6_speed);

            /* LCD (每200ms) */
            t6_elapsed++;
            if (t6_elapsed % 200 == 0) {
                lcd_set_cursor_big(0, 0);
                lcd_puts_big("T"); lcd_print_int_big((int32_t)(t6_elapsed / 1000));
                lcd_puts_big("s P"); lcd_print_int_big((int32_t)(b6_cm * 10));
                lcd_set_cursor_big(1, 0);
                lcd_puts_big("G"); lcd_print_int_big((int32_t)(t6_target * 10));
                lcd_puts_big(" S"); lcd_print_int_big(t6_step_pos);
            }
        }
    }

    /* ================================================================
     *  任务1: 视觉坐标校准 — 像素 + cm 对照
     *
     *  用法: 把球放到刻度 0/+5/-5 处, 读 LCD 上的 cm 值
     *        偏差大则调整 T1_CENTER_X 和 T1_PIX_PER_CM
     *        使得: 0刻度≈0cm, +5刻度≈+5cm, -5刻度≈-5cm
     * ================================================================ */
    if (task == 1) {
        #define T1_CENTER_X    325.0f   /* 球在0刻度时的像素X */
        #define T1_PIX_PER_CM  28.0f    /* 像素/cm */

        vision_init();
        lcd_set_cursor_big(0, 0); lcd_puts_big("T1 Calib");
        while (1) {
            if (g_ball_valid) {
                float t1_cm = (T1_CENTER_X - g_ball_x) / T1_PIX_PER_CM;
                int32_t t1_mm = (int32_t)(t1_cm * 10.0f);
                lcd_set_cursor_big(0, 0);
                lcd_puts_big("px:"); lcd_print_int_big((int32_t)g_ball_x);
                lcd_set_cursor_big(1, 0);
                lcd_puts_big("cm:"); lcd_print_int_big(t1_mm / 10);
                lcd_putc_big('.'); lcd_print_int_big(t1_mm % 10 >= 0 ? t1_mm % 10 : (-t1_mm) % 10);
            } else {
                lcd_set_cursor_big(1, 0);
                lcd_puts_big("No Ball");
            }
            delay_ms(100);
        }
    }
    if (task == 2) {
        #define T2_STOP_DIST  344.0f    /* 停车距离 cm */
        encoder_reset_distance();
        lcd_set_cursor_big(0, 0); lcd_puts_big("T2 Race");
        uint8_t  _f2[8];
        uint16_t t2 = 0;
        while (1) {
            delay_ms(10); t2++;
            sensor_read_all(_f2);
            encoder_update();
            float d = encoder_get_distance();

            /* 到达停车距离: 右轮先停, 左轮续转0.2s */
            if (d > T2_STOP_DIST) {
                sensor_set_mask(0xFF);
                motor_set(0, SPEED_LINE);        /* 右停左转 (motor_set内交换) */
                delay_ms(200);
                motor_brake(); beep(200);
                lcd_set_cursor_big(1, 0);
                lcd_puts_big("T:"); lcd_print_int_big((int32_t)(t2 / 100));
                lcd_puts_big("s OK");
                while (1) __WFI();
            }

            /* 330cm~停车: 屏蔽左3路(0,1,2)+右1路(7), 只用中间4路 */
            if (d > 330.0f)
                sensor_set_mask(0x78);           /* 0111 1000 = 通道 3,4,5,6 */
            else
                sensor_set_mask(0xFF);           /* 全通道 */

            if (sensor_is_line_detected()) line_follow();
            else motor_set(SPEED_LINE, SPEED_LINE);

            if (t2 % 20 == 0) {
                int32_t d_int = (int32_t)d;
                int32_t d_dec = (int32_t)((d - (float)d_int) * 10.0f);
                if (d_dec < 0) d_dec = 0;
                lcd_set_cursor_big(0, 0);
                lcd_print_int_big(d_int); lcd_putc_big('.');
                lcd_print_int_big(d_dec); lcd_puts_big("cm ");
                lcd_print_int_big((int32_t)(t2 / 100));
                lcd_puts_big("s  ");
                lcd_set_cursor_big(1, 0);
                lcd_puts_big("LINE ");
            }
            if (t2 % 50 == 0) DL_GPIO_togglePins(LED_G_PORT, LED_G_PIN);
        }
    }


    /* ================================================================
     *  任务3: 视觉 PID 闭环 — 保持小球在中心点 (位置式PID)
     *
     *  输出幅度 → 步进速率: |PID|大→快走, |PID|小→慢走, 死区→不走
     * ================================================================ */
    if (task == 3) {
        #define T3_CENTER_X    319.0f    /* 球中心像素X (校准后) */
        #define T3_PIX_PER_CM  28.7f     /* 像素/cm (校准后) */
        #define T3_BALL_CM()   ((T3_CENTER_X - g_ball_x) / T3_PIX_PER_CM)

        vision_init();
        stepper_init();
        ball_pid_init3();

        ball_pid_set_target3(0);     /* 目标: 中心点 */

        /* ---- UART3: PA26=TX(PINCM59) PA25=RX(PINCM55) 115200bps ---- */
        DL_GPIO_initPeripheralOutputFunction(IOMUX_PINCM59,
            IOMUX_PINCM59_PF_UART3_TX);
        DL_GPIO_initPeripheralInputFunction(IOMUX_PINCM55,
            IOMUX_PINCM55_PF_UART3_RX);
        DL_UART_Main_reset(UART3);
        DL_UART_Main_enablePower(UART3);
        DL_UART_Main_ClockConfig clk = {
            .clockSel    = DL_UART_MAIN_CLOCK_BUSCLK,
            .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1
        };
        DL_UART_Main_setClockConfig(UART3, &clk);
        DL_UART_Main_Config cfg = {
            .mode        = DL_UART_MAIN_MODE_NORMAL,
            .direction   = DL_UART_MAIN_DIRECTION_TX_RX,
            .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
            .parity      = DL_UART_MAIN_PARITY_NONE,
            .wordLength  = DL_UART_MAIN_WORD_LENGTH_8_BITS,
            .stopBits    = DL_UART_MAIN_STOP_BITS_ONE
        };
        DL_UART_Main_init(UART3, &cfg);
        DL_UART_Main_setOversampling(UART3, DL_UART_OVERSAMPLING_RATE_16X);
        DL_UART_Main_setBaudRateDivisor(UART3, 17, 23);
        DL_UART_Main_enable(UART3);

        delay_ms(500);  /* 先延时500ms, 不通电 */

        lcd_set_cursor_big(0, 0); lcd_puts_big("T3 Flick -5");
        delay_ms(500);
        lcd_clear(LCD_BLACK);

        /* ---- 阶段1: 开环 flick (250Hz) ---- */
        #define T3_FLICK_DELAY 128000   /* 250Hz */

        stepper_set_speed(-1);
        for (int i = 0; i < 42; i++) { stepper_step(); delay_cycles(T3_FLICK_DELAY); }
        delay_ms(165);

        stepper_set_speed(1);
        for (int i = 0; i < 150; i++) { stepper_step(); delay_cycles(T3_FLICK_DELAY); }
        delay_ms(380);

        stepper_set_speed(-1);
        for (int i = 0; i < 65; i++) { stepper_step(); delay_cycles(T3_FLICK_DELAY); }
        delay_ms(200);

        /* CW 10步 */
        stepper_set_speed(1);
        for (int i = 0; i < 10; i++) { stepper_step(); delay_cycles(T3_FLICK_DELAY); }

        /* ---- 阶段2: PID 锁定 -5cm ---- */
        ball_pid_set_target3(-5.0f);
        stepper_enable();
        lcd_set_cursor_big(1, 0); lcd_puts_big("PID +5->-5");

        int32_t  t3_step_pos = 0;
        uint32_t t3_ms = 0, t3_disp = 0, t3_prev_ms = 0;
        float t3_last_cm = 0, t3_vel = 0, t3_prev_cm = 0;
        uint8_t t3_deadband_count = 0;  /* 进死区次数 */
        uint8_t t3_was_in_deadband = 0;

        while (1) {
            t3_ms++;
            float t3_cm = T3_BALL_CM();
            if (!g_ball_valid) t3_cm = t3_last_cm; else t3_last_cm = t3_cm;

            if (g_ball_valid && t3_ms - t3_prev_ms >= 50) {
                float dt = (float)(t3_ms - t3_prev_ms) / 1000.0f;
                if (dt > 0) t3_vel = (t3_cm - t3_prev_cm) / dt;
                t3_prev_cm = t3_cm; t3_prev_ms = t3_ms;
            }

            long a3 = ball_pid_cascade_run3(t3_cm, t3_vel);

            /* 步数限位 */
            float t3_err_abs = fabsf(t3_cm - g_ball_pid3.target_cm);
            int32_t t3_lim = (int32_t)(t3_err_abs * 30);
            if (t3_lim < 0)   t3_lim = 0;
            if (t3_deadband_count >= 2) { if (t3_lim > 35) t3_lim = 35; }
            else                        { if (t3_lim > 40) t3_lim = 40; }
            if (t3_step_pos >=  t3_lim && a3 > 0) a3 = 0;
            if (t3_step_pos <= -t3_lim && a3 < 0) a3 = 0;

            /* 死区判断用视觉位置, 非PID输出 */
            float t3_err_to_target = g_ball_pid3.target_cm - t3_cm;
            uint8_t t3_in_dead = (fabsf(t3_err_to_target) < g_ball_pid3.pos_deadband);
            if (t3_in_dead && !t3_was_in_deadband) {
                t3_deadband_count++;
                t3_was_in_deadband = 1;
            }
            if (!t3_in_dead) t3_was_in_deadband = 0;

            /* 死区内归零 */
            if (a3 == 0 && t3_step_pos != 0) {
                a3 = (t3_step_pos > 0) ? -1 : 1;
            }

            if (t3_in_dead && t3_step_pos == 0 && t3_deadband_count >= 5) {
                stepper_disable();
                lcd_set_cursor_big(0,0); lcd_puts_big("T3 DONE!");
                while(1) __WFI();
            }

            if (a3 != 0) {
                stepper_set_speed(a3);
                stepper_step();
                t3_step_pos += (a3 > 0) ? 1 : -1;
                delay_cycles(64000);  /* 500Hz */
            } else {
                delay_ms(1);
            }

            t3_disp++;
            if (t3_disp >= 200) {
                t3_disp = 0;
                lcd_set_cursor(0, 0);
                lcd_puts("P:"); lcd_print_int((int32_t)(t3_cm * 10)); lcd_puts("mm");
                lcd_set_cursor(1, 0);
                lcd_puts("V:"); lcd_print_int((int32_t)(t3_vel * 10)); lcd_puts("mm/s");
                lcd_set_cursor(2, 0);
                lcd_puts("S:"); lcd_print_int(t3_step_pos);
                lcd_set_cursor(3, 0);
                lcd_puts("A:"); lcd_print_int((int32_t)a3);
            }
        }
    }

    /* ================================================================
     *  任务4: 赛题要求4 — 循迹A→B, 球稳定在中心, ≤8s, 误差≤1cm
     * ================================================================ */
    if (task == 4) {
        #define T4_CENTER_X    319.0f
        #define T4_PIX_PER_CM  28.7f
        #define T4_BALL_CM()   ((T4_CENTER_X - g_ball_x) / T4_PIX_PER_CM)

        vision_init();
        ball_pid_init();
        g_ball_pid.bias = 0;
        stepper_init();
        encoder_reset_distance();

        ball_pid_set_target(-0.1f);
        delay_ms(200);
        stepper_enable();

        /* PB1=TX 拉高, PB3=RX 下拉等待 */
        DL_GPIO_initDigitalOutput(IOMUX_PINCM13);  /* PB1 */
        DL_GPIO_setPins(GPIOB, DL_GPIO_PIN_1);
        DL_GPIO_enableOutput(GPIOB, DL_GPIO_PIN_1);
        DL_GPIO_initDigitalInputFeatures(IOMUX_PINCM16,
            DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_DOWN,
            DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);  /* PB3 */

        lcd_set_cursor_big(0, 0); lcd_puts_big("T4 A->B");

        uint8_t  _f4[8];
        uint16_t t4 = 0;
        uint32_t t4_prev_ms = 0;
        float t4_vel = 0, t4_prev_cm = 0, t4_last_cm = 0;
        int32_t t4_step_pos = 0;
        float    t4_speed_f = 10.0f;

        /* RX触发: 等1s→CCW20步(2s) */
        static uint8_t t4_rx_flag = 0;
        uint8_t t4_rx_step = 0;
        while (1) {
            t4++;

            /* RX: PB3高→等1s→CCW20步 */
            {static uint8_t ph=0;static uint16_t c=0,s=0;
            if(!t4_rx_flag&&DL_GPIO_readPins(GPIOB,DL_GPIO_PIN_3)){t4_rx_flag=1;ph=1;c=0;}
            if(ph==1){c++;if(c>=500){ph=2;c=0;s=0;}}
            if(ph==2){c++;if(c>=100){t4_rx_step=1;s++;c=0;if(s>=20)ph=3;}}}

            /* 小车加速: 初速12, 每秒+6, 30封顶 */
            if (t4_speed_f < 30.0f && t4 % 500 == 0) t4_speed_f += 4.0f;
            if (t4_speed_f > 30.0f) t4_speed_f = 36.0f;
            uint16_t t4_speed = (uint16_t)t4_speed_f;

            sensor_read_all(_f4);
            encoder_update();
            float d4 = encoder_get_distance();

            /* 球平衡 */
            float b4_cm = T4_BALL_CM();
            if (!g_ball_valid) b4_cm = t4_last_cm; else t4_last_cm = b4_cm;
            if (g_ball_valid && t4 - t4_prev_ms >= 50) {
                float dt = (float)(t4 - t4_prev_ms) / 1000.0f;
                if (dt > 0) t4_vel = (b4_cm - t4_prev_cm) / dt;
                t4_prev_cm = b4_cm; t4_prev_ms = t4;
            }

            long a4 = ball_pid_cascade_run(b4_cm, t4_vel);

            /* 步数限位 (同任务7) */
            float t4_err_abs = (b4_cm > 0) ? b4_cm : -b4_cm;
            int32_t t4_lim  = (int32_t)(t4_err_abs * 50);
            if (t4_lim < 0)   t4_lim = 0;
            if (t4_lim > 70) t4_lim = 70;
            if (t4_step_pos >=  t4_lim && a4 > 0) a4 = 0;
            if (t4_step_pos <= -t4_lim && a4 < 0) a4 = 0;

            /* 死区内归零 */
            if (a4 == 0 && t4_step_pos != 0) {
                a4 = (t4_step_pos > 0) ? -1 : 1;
            }

            if (t4_rx_step) {
                t4_rx_step = 0;
                stepper_set_speed(-1);
                stepper_step();
                delay_cycles(64000);
            } else if (a4 != 0) {
                stepper_set_speed(a4);
                stepper_step();
                t4_step_pos += (a4 > 0) ? 1 : -1;
                delay_cycles(64000);  /* 500Hz */
            } else {
                delay_ms(1);
            }

            /* 循迹(每10次更新) */
            if (t4 % 10 == 0) line_follow_speed(t4_speed);

            /* 到达B点: 150cm */
            if (d4 > 150.0f) {
                motor_brake();
                stepper_disable();
                lcd_set_cursor_big(0, 0); lcd_puts_big("T4 DONE!");
                lcd_set_cursor_big(1, 0);
                lcd_puts_big("T:"); lcd_print_int_big((int32_t)(t4 / 1000));
                lcd_puts_big("s ");
                lcd_puts_big("B:"); lcd_print_int_big((int32_t)(b4_cm * 10));
                lcd_puts_big("mm");
                while (1) __WFI();
            }

            if (t4 % 200 == 0) {
                lcd_set_cursor_big(0, 0);
                lcd_puts_big("D"); lcd_print_int_big((int32_t)d4);
                lcd_puts_big(" P"); lcd_print_int_big((int32_t)(b4_cm * 10));
                lcd_set_cursor_big(1, 0);
                lcd_puts_big("T"); lcd_print_int_big((int32_t)(t4 / 1000));
                lcd_puts_big("s S"); lcd_print_int_big(t4_step_pos);
            }
        }
    }

    /* 所有任务均在自己的 while(1) 中运行, 不会到达此处 */
    while (1) { __WFI(); }
}
