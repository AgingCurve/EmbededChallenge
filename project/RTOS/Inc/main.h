/**
  ******************************************************************************
  * @file    main.h
  * @brief   Shared types and BSP-level macros.
  *          Calibration / tunable constants live at the top of main.c.
  ******************************************************************************
  */

#ifndef __MAIN_H
#define __MAIN_H

#include "stm32f4xx_hal.h"
#include "stm324x9i_eval.h"
#include <stdbool.h>
#include <stdint.h>

/* ---------- BSP / encoder ISR ---------- */
#define READY                           3

/* ---------- Legacy ADCx macros (referenced by HAL_ADC_MspDeInit) ---------- */
#define ADCx                            ADC3
#define ADCx_CLK_ENABLE()               __ADC3_CLK_ENABLE();
#define ADCx_CHANNEL_GPIO_CLK_ENABLE()  __GPIOF_CLK_ENABLE()
#define ADCx_FORCE_RESET()              __ADC_FORCE_RESET()
#define ADCx_RELEASE_RESET()            __ADC_RELEASE_RESET()
#define ADCx_CHANNEL_PIN                GPIO_PIN_10
#define ADCx_CHANNEL_GPIO_PORT          GPIOF
#define ADCx_CHANNEL                    ADC_CHANNEL_8

/* ---------- FSM types ---------- */
typedef enum {
    START = 0,   /* Initial state */
    SEEK,        /* Search for a wall to track */
    ALIGNED,     /* Aligned with nearest wall (cruise) */
    LOCKED,      /* All sides under D_MIN — escape needed */
    EMERGENCY,   /* Imminent collision */
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
