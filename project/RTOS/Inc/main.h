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


/* ---------- FSM types ----------
 * Naive baseline FSM: Non-Island world + Dynamic Wall Tracking.
 * Pessimistic transitions — drop out of ALIGN at the first sign of
 * tracked-wall loss, re-enter ALIGN only with stable readings. */
typedef enum {
    INIT = 0,                /* sensor warm-up; wait for valid readings */
    SEEK,                    /* searching for any side wall */
    ALIGN_PROGRESS,          /* tracked wall locked; angle-corrected cruise */
    NON_ALIGN_PROGRESS,      /* no tracked wall; blind forward until one appears */
    EMERGENCY                /* imminent front collision -> pivot */
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
