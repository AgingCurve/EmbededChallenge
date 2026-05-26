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
