/**
  ******************************************************************************
  * @file    main.h
  * @brief   Baseline header — shared types/macros per README spec.
  ******************************************************************************
  */

#ifndef __MAIN_H
#define __MAIN_H

#include "stm32f4xx_hal.h"
#include "stm324x9i_eval.h"
#include <stdbool.h>
#include <stdint.h>

/* ---------- BSP / ADC legacy macros (kept from template) ---------- */
#define ADCx                            ADC3
#define ADCx_CLK_ENABLE()               __ADC3_CLK_ENABLE();
#define ADCx_CHANNEL_GPIO_CLK_ENABLE()  __GPIOF_CLK_ENABLE()
#define ADCx_FORCE_RESET()              __ADC_FORCE_RESET()
#define ADCx_RELEASE_RESET()            __ADC_RELEASE_RESET()
#define ADCx_CHANNEL_PIN                GPIO_PIN_10
#define ADCx_CHANNEL_GPIO_PORT          GPIOF
#define ADCx_CHANNEL                    ADC_CHANNEL_8
#define READY                           3

/* ---------- Sensing layer ---------- */
#define SAMPLE_N        7        /* per-signal sliding window (README) */

/* ---------- Motor layer ---------- */
#define PWM_PERIOD      20000    /* TIM ARR; duty range = [-PWM_PERIOD, +PWM_PERIOD] */
#define V_CRUISE        20000   /* full duty for now — back off after motion confirmed */

/* ---------- Control thresholds (minimum baseline; tune later) ---------- */
#define CTRL_PERIOD_MS    20
#define SENS_PERIOD_MS    20
#define IR_PERIOD_MS      20

#define D_TARGET          15     /* wall-follow target distance (cm)   */
#define D_MIN             8      /* lower safety bound (cm)            */
#define EMG_FRONT         6      /* emergency front threshold (cm)     */
#define IR_BUMP_HIGH      1100   /* IR ADC raw threshold for contact   */

/* ---------- Calibration / tunables ---------- */
#define US_TICKS_PER_CM   58     /* TIM3 IC diff -> cm divisor          */
#define EMG_FRONT_HYST    2      /* +cm  margin to clear EMERGENCY      */
#define IR_BUMP_HYST      100    /* -raw margin to clear EMERGENCY      */
#define TASK_WARMUP_MS    200    /* sensor task startup delay           */
#define CTRL_WARMUP_MS    300    /* control task startup delay          */
#define ADC_POLL_TIMEOUT  0xFF   /* HAL_ADC_PollForConversion timeout   */
#define TRIG_PULSE        2      /* TIM10 CH1 trigger pulse for HC-SR04 */

/* ---------- RTOS task params ---------- */
#define TASK_STACK_WORDS  256
#define PRIO_SENSOR       2
#define PRIO_IR           2
#define PRIO_CONTROL      1

/* ---------- FSM ---------- */
typedef enum {
    START = 0,   /* Initial state */
    SEEK,        /* Search for a wall to track */
    ALIGNED,     /* Aligned with nearest wall (cruise) */
    LOCKED,      /* All sides under D_MIN — escape needed */
    EMERGENCY,   /* Imminent collision / IR contact */
    INTERSECT,   /* Junction / corner decision */
    ENCOUNT,     /* Moving obstacle encountered */
    STOP         /* Halted */
} DriveState;

typedef enum {
    TRACK_LEFT = 0,
    TRACK_RIGHT
} TrackingSide;

/* canProgressDirection() return bitmask */
#define DIR_LEFT     0x01
#define DIR_FORWARD  0x02
#define DIR_RIGHT    0x04

#endif /* __MAIN_H */
