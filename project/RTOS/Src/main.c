/**
  ******************************************************************************
  * @file    main.c
  * @brief   Baseline — peripheral init + RTOS tasks per README spec.
  *
  *  Tasks (README "Functions"):
  *    UltraSonicTask  : median7 / stddev7 / SensorTask
  *    IR_Task         : IR_Task
  *    ControlTask     : Motor_Drive / Motor_Stop / switchTracking /
  *                      angleAdjusting / angleCalculate /
  *                      canProgressDirection / isEmergency /
  *                      emergencyResolved / ControlTask
  ******************************************************************************
  */

#include "main.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "cmsis_os.h"

/* ===========================================================================
 *  Calibration / tunables  (tune everything here)
 * =========================================================================== */
/* Sensing */
#define SAMPLE_N          7        /* sliding window length per signal */
#define US_TICKS_PER_CM   58       /* TIM3 IC diff -> cm divisor       */

/* Task timing (ms) */
#define CTRL_PERIOD_MS    20
#define SENS_PERIOD_MS    20
#define IR_PERIOD_MS      20
#define DBG_PERIOD_MS     500       /* DebugTask periodic snapshot cadence */
#define TASK_WARMUP_MS    200
#define CTRL_WARMUP_MS    300

/* Distance thresholds (cm) */
#define D_TARGET          20       /* wall-follow target distance */
#define D_MIN             10        /* lower safety bound          */
#define D_OPEN            150       /* > D_OPEN => "no wall on this side" */
#define EMG_FRONT         9        /* emergency front threshold (cm) */
#define EMG_FRONT_HYST    2        /* +cm margin to clear EMERGENCY */
#define IR_BUMPER_THRESH  1000     /* raw ADC; ir_left/right BELOW this => bumper triggered.
                                    * Sensor is inverse (closer = lower ADC). Baseline far ~2500-3000. */
#define EMERG_IR_DEG      30       /* rotation angle when IR bumper triggers EMERGENCY (small nudge) */
#define EMERG_US_DEG      90       /* rotation angle when front ultrasonic triggers EMERGENCY (full pivot) */

/* Motor PWM */
#define PWM_PERIOD          20000
#define V_CRUISE            15000   /* duty for all forward drive (SEEK / ALIGN / NON_ALIGN) */
#define V_TURN              20000   /* full duty for max pivot torque */

/* Iterative pivot — the only rotation primitive.
 * 90° = PIVOT_SUBSTEPS_90_x × PIVOT_SUBSTEP_TICKS encoder ticks total.
 * L/R split because the two wheels have slightly different effective
 * ticks-per-degree (motor/encoder asymmetry). Tune by 4×90° round-trip
 * calibration (CALIB_PIVOT mode); keep tick count fixed. */
#define PIVOT_SUBSTEP_TICKS       30   /* encoder ticks per micro-pivot (~3°) */
#define PIVOT_SUBSTEPS_90_L       24   /* micro-pivots for 90° LEFT  (calib 2026-05-27 final) */
#define PIVOT_SUBSTEPS_90_R       25   /* micro-pivots for 90° RIGHT (calib 2026-05-27 final) */
#define PIVOT_PAUSE_MS            10   /* brief stop between micro-pivots */
#define PIVOT_SUBSTEP_TIMEOUT_MS  200  /* per-substep safety */
#define POST_TURN_SETTLE_MS       300  /* let SensorTask median refresh after pivot */

/* Angle correction (arctan-based, event-driven via rotate_iterative) */
#define ANGLE_HISTORY_N      10   /* ring buffer length (10 * 20ms = 200ms accumulator) */
#define ANGLE_CORRECT_DEG    10   /* below this magnitude, skip correction */
#define MIN_DF_DELTA_CM      2    /* |dF_old - dF_new| < this -> ratio unreliable */

/* HC-SR04 trigger */
#define TRIG_PULSE        2

/* ADC */
#define ADC_POLL_TIMEOUT  0xFF

/* ALIGN_PROGRESS side-wall avoidance: when a side reads < D_MIN cm, stop and
 * pivot away by this many degrees (replaces the previous half-speed veer-off). */
#define ROTATE_VEER_DEG   30

/* Pivot calibration mode — when 1, ControlTask skips the FSM and runs a
 * 4x90° round-trip (right then left) so PIVOT_SUBSTEPS_90 can be measured
 * against floor markings. Set to 0 for normal driving. */
#define CALIB_PIVOT       0

/* IR calibration mode — when 1, ControlTask skips the FSM, motors stay
 * stopped, and IR readings stream to UART at 100ms cadence so a human
 * can wave obstacles in front of each sensor to find IR_BUMPER_THRESH. */
#define CALIB_IR          0

/* ===========================================================================
 *  Peripherals
 * =========================================================================== */
TIM_HandleTypeDef    TimHandle1, TimHandle2, TimHandle3, TimHandle4;
TIM_IC_InitTypeDef   sICConfig;
TIM_OC_InitTypeDef   sConfig1, sConfig2, sConfig3;

uint32_t uwPrescalerValue = 0;
uint16_t motorInterrupt1 = 0;     /* Right encoder */
uint16_t motorInterrupt2 = 0;     /* Left  encoder */
uint8_t  encoder_right  = READY;
uint8_t  encoder_left   = READY;

/* Ultrasonic input-capture raw diffs (TIM3 CH2/3/4) */
uint32_t uwIC2Value1 = 0, uwIC2Value2 = 0, uwDiffCapture1 = 0;  /* Right */
uint32_t uwIC2Value3 = 0, uwIC2Value4 = 0, uwDiffCapture2 = 0;  /* Front */
uint32_t uwIC2Value5 = 0, uwIC2Value6 = 0, uwDiffCapture3 = 0;  /* Left  */
uint32_t uwFrequency = 0;

ADC_HandleTypeDef       AdcHandle1, AdcHandle2, AdcHandle3;
ADC_ChannelConfTypeDef  adcConfig1, adcConfig2, adcConfig3;

__IO uint32_t uhADCxRight;
__IO uint32_t uhADCxForward;
__IO uint32_t uhADCxLeft;

extern UART_HandleTypeDef UartHandle1, UartHandle2;

/* ===========================================================================
 *  Shared sensing state
 * =========================================================================== */
static int us_buf_F[SAMPLE_N], us_buf_L[SAMPLE_N], us_buf_R[SAMPLE_N];
static int us_idx = 0;

int dF = 0, dL = 0, dR = 0;            /* filtered distances (cm)        */
int sF = 0, sL = 0, sR = 0;            /* per-window stddev (confidence) */
int dF_prev = 0, dL_prev = 0, dR_prev = 0;

/* Angle-correction ring buffer (filled by SensorTask each tick).
 * angleCalculate() compares the oldest slot to the newest reading. */
static int hist_dF[ANGLE_HISTORY_N];
static int hist_dL[ANGLE_HISTORY_N];
static int hist_dR[ANGLE_HISTORY_N];
static int hist_idx   = 0;
static int hist_count = 0;             /* warm-up counter; valid when == N */

int ir_floor = 0, ir_left = 0, ir_right = 0;

DriveState   state = INIT;
TrackingSide side  = TRACK_RIGHT;

/* EMERGENCY commit state: lock turn direction until SEEK is stable for a while */
static bool emerg_committed = false;
static bool emerg_turn_left = false;
static int  emerg_turn_deg  = EMERG_US_DEG;   /* committed rotation magnitude */
static int  seek_clear_ticks = 0;
#define EMERG_RELEASE_TICKS  25     /* ~500ms of clear SEEK -> release commit */

/* ===========================================================================
 *  Forward declarations
 * =========================================================================== */
static void SystemClock_Config(void);
static void EXTILine_Config(void);
static void Error_Handler(void);

/* Sensing */
int  median7(int *a);
int  stddev7(int *a);
void angleHistoryFlush(void);
void SensorTask(void *arg);
void IR_Task(void *arg);

/* Debug */
void DebugTask(void *arg);

/* Control */
void    Motor_Drive(int v_left, int v_right);
void    Motor_Stop(void);
void    rotate_iterative(int degrees, bool left);
bool    switchTracking(void);
void    angleAdjusting(void);
int     angleCalculate(void);
uint8_t canProgressDirection(void);
bool    isEmergency(void);
bool    emergencyResolved(void);
void    ControlTask(void *arg);

/* ===========================================================================
 *  printf -> UART
 * =========================================================================== */
#ifdef __GNUC__
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
/**
 * @brief  Redirects single-character stdio output to UART1 so printf works.
 * @param  ch  Character to emit (passed by stdio runtime).
 * @return The same character, as expected by stdio.
 */
PUTCHAR_PROTOTYPE
{
    HAL_UART_Transmit(&UartHandle1, (uint8_t *)&ch, 1, 0xFFFF);
    return ch;
}

/* ===========================================================================
 *  Sensing Layer — UltraSonicTask
 *  7-sample window per signal -> filtered value (median) + confidence (stddev).
 *  Rate of change is derived from (d* - d*_prev).
 * =========================================================================== */

/**
 * @brief  Returns the median of SAMPLE_N (=7) integers via insertion sort.
 * @param  a  Pointer to the sliding-window buffer (SAMPLE_N entries).
 * @return The middle element after sorting a local copy (input is untouched).
 */
int median7(int *a)
{
    int tmp[SAMPLE_N];
    for (int i = 0; i < SAMPLE_N; i++) tmp[i] = a[i];
    for (int i = 1; i < SAMPLE_N; i++) {
        int v = tmp[i], j = i - 1;
        while (j >= 0 && tmp[j] > v) { tmp[j + 1] = tmp[j]; j--; }
        tmp[j + 1] = v;
    }
    return tmp[SAMPLE_N / 2];
}

/**
 * @brief  Integer-only standard deviation of SAMPLE_N (=7) samples.
 * @param  a  Pointer to the sliding-window buffer.
 * @return floor(sqrt(variance)); used as a per-sensor noise/confidence proxy.
 */
int stddev7(int *a)
{
    long sum = 0;
    for (int i = 0; i < SAMPLE_N; i++) sum += a[i];
    int mean = (int)(sum / SAMPLE_N);
    long var = 0;
    for (int i = 0; i < SAMPLE_N; i++) {
        long d = a[i] - mean;
        var += d * d;
    }
    var /= SAMPLE_N;
    int s = 0;
    while ((long)(s + 1) * (s + 1) <= var) s++;
    return s;
}

/**
 * @brief  Resets the angle-correction ring buffer to "warming up" state.
 * @note   Called whenever past samples become semantically invalid:
 *         after rotate_iterative() (heading changed) or on side switch.
 *         Forces angleCalculate() to return 0 until N fresh samples are pushed.
 */
void angleHistoryFlush(void)
{
    hist_idx   = 0;
    hist_count = 0;
}

/**
 * @brief  RTOS task — samples HC-SR04 trio, filters them, feeds the angle buffer.
 * @param  arg  Unused (FreeRTOS task signature).
 * @note   Period = SENS_PERIOD_MS. Each tick: shifts US_TICKS_PER_CM-scaled
 *         raw IC diffs into ring buffers, recomputes (dF, dL, dR) via median7
 *         and (sF, sL, sR) via stddev7, then pushes the filtered triple into
 *         the arctan history buffer (hist_d*).
 */
void SensorTask(void *arg)
{
    (void)arg;
    osDelay(TASK_WARMUP_MS);
    for (;;) {
        us_buf_F[us_idx] = (int)(uwDiffCapture2 / US_TICKS_PER_CM);
        us_buf_L[us_idx] = (int)(uwDiffCapture3 / US_TICKS_PER_CM);
        us_buf_R[us_idx] = (int)(uwDiffCapture1 / US_TICKS_PER_CM);
        us_idx = (us_idx + 1) % SAMPLE_N;

        dF_prev = dF; dL_prev = dL; dR_prev = dR;
        dF = median7(us_buf_F);
        dL = median7(us_buf_L);
        dR = median7(us_buf_R);
        sF = stddev7(us_buf_F);
        sL = stddev7(us_buf_L);
        sR = stddev7(us_buf_R);

        /* Push filtered values into angle-correction ring buffer. */
        hist_dF[hist_idx] = dF;
        hist_dL[hist_idx] = dL;
        hist_dR[hist_idx] = dR;
        hist_idx = (hist_idx + 1) % ANGLE_HISTORY_N;
        if (hist_count < ANGLE_HISTORY_N) hist_count++;

        osDelay(SENS_PERIOD_MS);
    }
}

/* ===========================================================================
 *  Sensing Layer — IR_Task
 * =========================================================================== */
/**
 * @brief  RTOS task — polls the three IR ADC channels (left, right, floor).
 * @param  arg  Unused (FreeRTOS task signature).
 * @note   Period = IR_PERIOD_MS. Stores raw 12-bit ADC values into
 *         ir_left/ir_right/ir_floor for ControlTask diagnostics.
 */
void IR_Task(void *arg)
{
    (void)arg;
    osDelay(TASK_WARMUP_MS);
    for (;;) {
        HAL_ADC_Start(&AdcHandle1);
        HAL_ADC_PollForConversion(&AdcHandle1, ADC_POLL_TIMEOUT);
        ir_left  = (int)HAL_ADC_GetValue(&AdcHandle1);

        HAL_ADC_Start(&AdcHandle2);
        HAL_ADC_PollForConversion(&AdcHandle2, ADC_POLL_TIMEOUT);
        ir_right = (int)HAL_ADC_GetValue(&AdcHandle2);

        HAL_ADC_Start(&AdcHandle3);
        HAL_ADC_PollForConversion(&AdcHandle3, ADC_POLL_TIMEOUT);
        ir_floor = (int)HAL_ADC_GetValue(&AdcHandle3);

        osDelay(IR_PERIOD_MS);
        printf("\r\n[IR_Bumping]Left : %d, Right : %d", ir_left, ir_right);
    }
}

/* ===========================================================================
 *  Debug Layer — DebugTask
 *  Two outputs:
 *    1) Edge log on every FSM state transition (printed once per change).
 *    2) Periodic snapshot every DBG_PERIOD_MS with distances, stddev, side,
 *       emergency-commit info, and IR readings.
 *  Lowest priority so it never starves Sensor/IR/Control.
 * =========================================================================== */
void DebugTask(void *arg)
{
    (void)arg;
    static const char *state_name[] = {
        "INIT", "SEEK", "ALIGN_PROG", "NON_ALIGN_PROG", "EMERGENCY"
    };
    DriveState prev_state = (DriveState)0xFF;   /* force first-tick edge log */
    TrackingSide prev_side = side;
    uint32_t tick = 0;

    osDelay(TASK_WARMUP_MS);
    for (;;) {
        if (state != prev_state) {
            printf("\r\n[DBG-EDGE] %s -> %s  dF=%d dL=%d dR=%d side=%s",
                   (prev_state == (DriveState)0xFF) ? "?" : state_name[prev_state],
                   state_name[state],
                   dF, dL, dR,
                   side == TRACK_RIGHT ? "R" : "L");
            prev_state = state;
        }
        if (side != prev_side) {
            printf("\r\n[DBG-SIDE] %s -> %s",
                   prev_side == TRACK_RIGHT ? "R" : "L",
                   side      == TRACK_RIGHT ? "R" : "L");
            prev_side = side;
        }

        printf("\r\n[DBG #%lu] st=%s side=%s "
               "d(F/L/R)=%d/%d/%d s(F/L/R)=%d/%d/%d "
               "emg=%c commit=%c hist=%d ir(L/R/F)=%d/%d/%d",
               (unsigned long)tick++,
               state_name[state],
               side == TRACK_RIGHT ? "R" : "L",
               dF, dL, dR,
               sF, sL, sR,
               isEmergency() ? '1' : '0',
               emerg_committed ? (emerg_turn_left ? 'L' : 'R') : '-',
               hist_count,
               ir_left, ir_right, ir_floor);

        osDelay(DBG_PERIOD_MS);
    }
}

/* ===========================================================================
 *  Motor Layer
 *  v_left / v_right ∈ [-PWM_PERIOD, +PWM_PERIOD].  Sign = direction.
 *  TIM8 drives right motor (CH1=fwd, CH2=rev); TIM4 drives left (CH2=fwd, CH1=rev).
 * =========================================================================== */

/**
 * @brief  Sets PWM duty on both motors; sign encodes direction.
 * @param  v_left   Signed duty for left  wheel; positive = forward.
 * @param  v_right  Signed duty for right wheel; positive = forward.
 * @note   Magnitudes are clamped to PWM_PERIOD. Idle channel gets CCR=0 (PWM1
 *         mode outputs LOW), the active channel gets |v|. Channels are pre-
 *         started in main(); this function only updates compare registers.
 */
void Motor_Drive(int v_left, int v_right)
{
    /* PWM channels are pre-started in main(); we just gate by CCR.
     * CCR = 0 with PWM1 mode -> output always LOW (channel idle).
     * TIM8: CH1=right fwd, CH2=right rev
     * TIM4: CH2=left  fwd, CH1=left  rev
     */
    int aL = v_left  < 0 ? -v_left  : v_left;
    int aR = v_right < 0 ? -v_right : v_right;
    if (aL > PWM_PERIOD) aL = PWM_PERIOD;
    if (aR > PWM_PERIOD) aR = PWM_PERIOD;

    if (v_right >= 0) { TIM8->CCR1 = aR; TIM8->CCR2 = 0;  }
    else              { TIM8->CCR1 = 0;  TIM8->CCR2 = aR; }
    if (v_left  >= 0) { TIM4->CCR2 = aL; TIM4->CCR1 = 0;  }
    else              { TIM4->CCR2 = 0;  TIM4->CCR1 = aL; }
}

/**
 * @brief  Coasts the robot — both motors get zero duty without disabling PWM.
 * @note   Uses Motor_Drive(0, 0) so the PWM channels stay enabled and the
 *         next non-zero call takes effect immediately. HAL_TIM_PWM_Stop()
 *         would have to be matched by a Start() and is intentionally avoided.
 */
void Motor_Stop(void)
{
    Motor_Drive(0, 0);
}

/**
 * @brief  Closed-loop in-place pivot driven by encoder ticks, in micro-steps.
 * @param  degrees  Magnitude of rotation in degrees (positive).
 * @param  left     true => pivot CCW (left turn); false => pivot CW (right).
 * @note   Decomposes the turn into `PIVOT_SUBSTEPS_90 * degrees / 90`
 *         micro-pivots, each consuming PIVOT_SUBSTEP_TICKS ticks on the wheel
 *         that goes forward for this direction. Stops between micro-steps to
 *         null out inertia. Settles POST_TURN_SETTLE_MS so the median filter
 *         refreshes, then calls angleHistoryFlush() — every rotation
 *         invalidates the arctan ring buffer's heading assumption.
 */
void rotate_iterative(int degrees, bool left)
{
    int subs90 = left ? PIVOT_SUBSTEPS_90_L : PIVOT_SUBSTEPS_90_R;
    int substeps = (subs90 * degrees) / 90;
    if (substeps <= 0) return;

    /* pivot LEFT  (CCW): right wheel forward -> use motorInterrupt1
     * pivot RIGHT (CW):  left  wheel forward -> use motorInterrupt2 */
    volatile uint16_t *enc = left ? &motorInterrupt1 : &motorInterrupt2;

    int total_ms = 0;
    for (int i = 0; i < substeps; i++) {
        Motor_Stop();
        osDelay(PIVOT_PAUSE_MS);
        *enc = 0;
        if (left) Motor_Drive(-V_TURN,  V_TURN);
        else      Motor_Drive( V_TURN, -V_TURN);
        /* Encoder ISR may ++ or -- depending on wheel-direction polarity
         * (mirrored motors on differential drive). Cast to int16_t so an
         * underflow (0 -> 65535) reads as -1, and compare |displacement|. */
        int t = 0;
        while (t < PIVOT_SUBSTEP_TIMEOUT_MS) {
            int16_t s = (int16_t)*enc;
            if (s < 0) s = -s;
            if (s >= PIVOT_SUBSTEP_TICKS) break;
            osDelay(1);
            t++;
        }
        Motor_Stop();
        total_ms += t + PIVOT_PAUSE_MS;
    }
    printf("\r\n>> ROT-ITER %s deg=%d substeps=%d ms=%d",
           left ? "L" : "R", degrees, substeps, total_ms);
    osDelay(POST_TURN_SETTLE_MS);
    angleHistoryFlush();   /* heading changed -> all prior samples invalid */
}

/* ===========================================================================
 *  Control Layer — helpers
 * =========================================================================== */

/**
 * @brief  Sticky dynamic wall tracking — only swaps `side` when current is lost.
 * @return true if a usable tracked wall is in place after the call,
 *         false if both sides lack a wall within D_TARGET.
 * @note   Sticky policy: a working track is never preempted. If the current
 *         side fails but the opposite side has a wall, `side` flips and the
 *         angle history is flushed (the heading-relative correlation we were
 *         tracking is now against a different wall).
 */
bool switchTracking(void)
{
    bool cur_ok, alt_ok;
    if (side == TRACK_RIGHT) {
        cur_ok = (dR > 0 && dR <= D_TARGET);
        alt_ok = (dL > 0 && dL <= D_TARGET);
    } else {
        cur_ok = (dL > 0 && dL <= D_TARGET);
        alt_ok = (dR > 0 && dR <= D_TARGET);
    }

    if (cur_ok) return true;
    if (alt_ok) {
        side = (side == TRACK_RIGHT) ? TRACK_LEFT : TRACK_RIGHT;
        angleHistoryFlush();
        return true;
    }
    return false;
}

/* Source sensor of the most recent non-zero angleCalculate() result.
 * Consumed by angleAdjusting() to pick rotation direction. */
static TrackingSide angle_source = TRACK_RIGHT;

/**
 * @brief  Heading deviation in degrees, derived by arctan over a 200ms window.
 * @return Signed degrees; 0 means "skip correction this tick". Sign is
 *         per-wall (NOT unified): from R sensor, +deg => tilted toward right
 *         wall; from L sensor, +deg => tilted toward left wall. The source
 *         sensor of a non-zero result is left in `angle_source` for the
 *         caller (angleAdjusting) to interpret.
 * @note   Independent of `side` — picks dR first if both current and oldest
 *         dR samples are valid; falls back to dL otherwise. Returns 0 when
 *         the ring buffer isn't warm yet, |ΔdF| < MIN_DF_DELTA_CM (robot
 *         essentially stationary / no front motion to differentiate), or
 *         neither side has paired valid samples.
 *           tan(θ_R) = (dR_old - dR_new) / (dF_old - dF_new)
 *           tan(θ_L) = (dL_old - dL_new) / (dF_old - dF_new)
 */
int angleCalculate(void)
{
    if (hist_count < ANGLE_HISTORY_N) return 0;

    int oldest = hist_idx;             /* slot about to be overwritten = oldest */
    int dF_old = hist_dF[oldest];
    int denom  = dF_old - dF;
    int adenom = denom < 0 ? -denom : denom;
    if (adenom < MIN_DF_DELTA_CM) return 0;

    int dR_old = hist_dR[oldest];
    int dL_old = hist_dL[oldest];
    float denom_f = (float)denom;
    float tan_theta;

    if (dR > 0 && dR_old > 0) {
        tan_theta    = (float)(dR_old - dR) / denom_f;
        angle_source = TRACK_RIGHT;
    } else if (dL > 0 && dL_old > 0) {
        tan_theta    = (float)(dL_old - dL) / denom_f;
        angle_source = TRACK_LEFT;
    } else {
        return 0;
    }

    float theta_rad = atanf(tan_theta);
    return (int)(theta_rad * 57.2957795f);   /* rad -> deg */
}

/**
 * @brief  Event-driven heading correction — runs once per ALIGN_PROGRESS tick.
 * @note   |θ| < ANGLE_CORRECT_DEG  =>  straight cruise at V_CRUISE (most ticks).
 *         |θ| >= ANGLE_CORRECT_DEG =>  stop, in-place pivot by |θ|, then resume
 *         (rotate_iterative auto-flushes the history). Rotation direction is
 *         picked from `angle_source` so the per-wall sign convention from
 *         angleCalculate() is interpreted correctly.
 */
void angleAdjusting(void)
{
    int deg = angleCalculate();
    int mag = deg < 0 ? -deg : deg;

    if (mag < ANGLE_CORRECT_DEG) {
        Motor_Drive(V_CRUISE, V_CRUISE);
        return;
    }

    bool turn_left = (angle_source == TRACK_RIGHT) ? (deg > 0) : (deg < 0);

    printf("\r\n>> ANG-CORR theta=%d src=%s turn=%s",
           deg, angle_source == TRACK_RIGHT ? "R" : "L", turn_left ? "L" : "R");

    Motor_Stop();
    osDelay(50);
    rotate_iterative(mag, turn_left);
}

/**
 * @brief  Bitmask of directions safe to take/turn into right now.
 * @return OR of DIR_FORWARD / DIR_LEFT / DIR_RIGHT bits; set means safe.
 * @note   "Safe" = the corresponding sensor either sees nothing (==0) or
 *         reads a distance > EMG_FRONT. Used in EMERGENCY to pick which
 *         way to rotate so the robot doesn't immediately re-enter EMERGENCY.
 */
uint8_t canProgressDirection(void)
{
    uint8_t mask = 0;
    if (dF == 0 || dF > EMG_FRONT) mask |= DIR_FORWARD;
    if (dL == 0 || dL > EMG_FRONT) mask |= DIR_LEFT;
    if (dR == 0 || dR > EMG_FRONT) mask |= DIR_RIGHT;
    return mask;
}

/**
 * @brief  True when a forward obstacle is close enough to require evasion.
 * @return dF in (0, EMG_FRONT] OR a side IR bumper reads below IR_BUMPER_THRESH
 *         (inverse sensor: lower ADC = closer).
 */
bool isEmergency(void)
{
    return (dF > 0 && dF <= EMG_FRONT) ||
           (ir_left  > 0 && ir_left  < IR_BUMPER_THRESH) ||
           (ir_right > 0 && ir_right < IR_BUMPER_THRESH);
}

/**
 * @brief  True when the front has cleared past the hysteresis threshold.
 * @return dF > EMG_FRONT + EMG_FRONT_HYST. Used to release EMERGENCY commit
 *         once the robot is solidly back into safe territory.
 */
bool emergencyResolved(void)
{
    return (dF > EMG_FRONT + EMG_FRONT_HYST);
}

/* ===========================================================================
 *  Control Layer — FSM
 * =========================================================================== */
/**
 * @brief  RTOS task — runs the 5-state drive FSM each CTRL_PERIOD_MS.
 * @param  arg  Unused (FreeRTOS task signature).
 * @note   States: INIT -> SEEK -> ALIGN_PROGRESS <-> NON_ALIGN_PROGRESS;
 *         EMERGENCY preempts any state when isEmergency() trips and returns
 *         to SEEK after a committed 90° rotate_iterative(). ALIGN_PROGRESS
 *         is the only state that calls angleAdjusting(). LED1 toggles every
 *         ~500ms and a diagnostic line is printed at the same cadence.
 */
void ControlTask(void *arg)
{
    (void)arg;
    static const char *state_name[] = {
        "INIT","SEEK","ALIGN_PROG","NON_ALIGN_PROG","EMERGENCY"
    };
    uint32_t tick = 0;

    osDelay(CTRL_WARMUP_MS);

#if CALIB_PIVOT
    /* Calibration: 4×90° left only (right side locked from prior calibration).
     * Mark heading on floor before boot, compare on stop. Tune PIVOT_SUBSTEPS_90_L. */
    osDelay(1000);
    for (int i = 0; i < 4; i++) {
        printf("\r\n>> CALIB L %d/4", i + 1);
        rotate_iterative(90, true);
        osDelay(1000);
    }
    printf("\r\n>> CALIB DONE");
    Motor_Stop();
    for (;;) osDelay(1000);   /* trap — flip CALIB_PIVOT to 0 and reflash */
#endif

#if CALIB_IR
    /* Motors off, stream IR readings to UART at 10 Hz.
     * Move obstacles past each sensor and note the ADC at "this is too close".
     * That value -> set IR_BUMPER_THRESH (when re-enabling IR bumper logic). */
    Motor_Stop();
    printf("\r\n>> CALIB IR START");
    for (;;) {
        printf("\r\n>> IR L=%d R=%d F=%d", ir_left, ir_right, ir_floor);
        osDelay(100);
    }
#endif

    for (;;) {
        /* Top-level guard: any state can be preempted by EMERGENCY. */
        if (isEmergency()) state = EMERGENCY;

        switch (state) {
            case INIT:
                /* Wait for first valid front + any side reading. */
                if (dF > 0 && (dL > 0 || dR > 0)) state = SEEK;
                break;

            case SEEK: {
                /* No tracked wall yet — cruise forward, veer AWAY from a side that gets dangerously close.
                 * Differential drive: slowing the FAR-side wheel pivots the robot away from the near wall. */
                int vL = V_CRUISE, vR = V_CRUISE;
                if (dR > 0 && dR < D_MIN) vL = V_CRUISE / 2;   /* R wall close -> slow L -> veer LEFT */
                if (dL > 0 && dL < D_MIN) vR = V_CRUISE / 2;   /* L wall close -> slow R -> veer RIGHT */
                Motor_Drive(vL, vR);

                if (switchTracking()) state = ALIGN_PROGRESS;
                if (++seek_clear_ticks >= EMERG_RELEASE_TICKS) emerg_committed = false;
            } break;

            case ALIGN_PROGRESS: {
                /* Sticky tracking: only swaps if current side is lost.
                 * Returns false -> no usable wall on either side -> demote. */
                if (!switchTracking()) {
                    state = NON_ALIGN_PROGRESS;
                    break;
                }

                /* Side-wall avoidance: stop and pivot away decisively (no smooth veer). */
                if (dR > 0 && dR < D_MIN) {
                    printf("\r\n>> VEER L deg=%d dR=%d", ROTATE_VEER_DEG, dR);
                    Motor_Stop();
                    osDelay(50);
                    rotate_iterative(ROTATE_VEER_DEG, true);   /* R wall close -> pivot LEFT */
                } else if (dL > 0 && dL < D_MIN) {
                    printf("\r\n>> VEER R deg=%d dL=%d", ROTATE_VEER_DEG, dL);
                    Motor_Stop();
                    osDelay(50);
                    rotate_iterative(ROTATE_VEER_DEG, false);  /* L wall close -> pivot RIGHT */
                } else {
                    Motor_Drive(V_CRUISE, V_CRUISE);
                }

                if (++seek_clear_ticks >= EMERG_RELEASE_TICKS) emerg_committed = false;
            } break;

            case NON_ALIGN_PROGRESS: {
                /* No tracked wall — drive straight; promote back to ALIGN once one appears. */
                Motor_Drive(V_CRUISE, V_CRUISE);

                if (switchTracking()) state = ALIGN_PROGRESS;
                if (++seek_clear_ticks >= EMERG_RELEASE_TICKS) emerg_committed = false;
            } break;

            case EMERGENCY: {
                /* Pick rotation direction. IR bumper takes priority — if a side
                 * bumper trips (lower ADC = closer), rotate AWAY from it. The
                 * side with the LOWER reading is closer when both fire.
                 * Otherwise fall back to front-wall reasoning:
                 *
                 *   no tracked wall                 -> default right turn
                 *   both sides progressable         -> rotate so the previously-blocking
                 *                                      front wall ends up on `side`
                 *   only one side progressable      -> rotate that way
                 *   none progressable (dead end)    -> default right
                 *
                 * Commit on first entry; reuse until SEEK clears. */
                bool first = !emerg_committed;
                if (first) {
                    bool ir_l_hit = (ir_left  > 0 && ir_left  < IR_BUMPER_THRESH);
                    bool ir_r_hit = (ir_right > 0 && ir_right < IR_BUMPER_THRESH);

                    if (ir_l_hit || ir_r_hit) {
                        /* IR bumper: small nudge (sensors near front corners). */
                        emerg_turn_deg = EMERG_IR_DEG;
                        if (ir_l_hit && ir_r_hit) emerg_turn_left = (ir_right < ir_left);  /* both: away from lower (closer) */
                        else if (ir_l_hit)        emerg_turn_left = false;                 /* left bumper  -> turn right */
                        else                      emerg_turn_left = true;                  /* right bumper -> turn left  */
                    } else {
                        /* Front ultrasonic wall: full 90° pivot to follow new corridor. */
                        emerg_turn_deg = EMERG_US_DEG;
                        uint8_t mask = canProgressDirection();
                        bool can_left  = (mask & DIR_LEFT)  != 0;
                        bool can_right = (mask & DIR_RIGHT) != 0;
                        bool has_track = (dR > 0 && dR <= D_TARGET) ||
                                         (dL > 0 && dL <= D_TARGET);

                        if (!has_track) {
                            emerg_turn_left = false;                     /* default right */
                        } else if (can_left && can_right) {
                            emerg_turn_left = (side == TRACK_RIGHT);     /* front wall -> new `side` */
                        } else if (can_left) {
                            emerg_turn_left = true;
                        } else if (can_right) {
                            emerg_turn_left = false;
                        } else {
                            emerg_turn_left = false;                     /* dead end */
                        }
                    }
                    emerg_committed = true;
                }
                seek_clear_ticks = 0;
                printf("\r\n>> EMERG %s deg=%d commit=%s dF=%d dL=%d dR=%d ir(L/R)=%d/%d",
                       emerg_turn_left ? "LEFT" : "RIGHT", emerg_turn_deg,
                       first ? "NEW" : "KEEP", dF, dL, dR, ir_left, ir_right);
                Motor_Stop();
                osDelay(50);
                rotate_iterative(emerg_turn_deg, emerg_turn_left);
                state = SEEK;
            } break;
        }
        osDelay(CTRL_PERIOD_MS);
    }
}

/* ===========================================================================
 *  main
 * =========================================================================== */
/**
 * @brief  System entry point: brings up peripherals, then launches the RTOS.
 * @return Never returns in normal operation. If vTaskStartScheduler() falls
 *         through, LED2 lights solid as a fault indicator and the loop spins.
 * @note   Init order: HAL_Init -> SystemClock_Config -> UART/LED -> motor PWM
 *         (TIM8/TIM4) -> EXTI (encoders) -> ultrasonic IC (TIM3) + trigger
 *         (TIM10) -> three ADCs for IR -> task creation -> scheduler start.
 */
int main(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    HAL_Init();
    SystemClock_Config();
    BSP_COM1_Init();
    BSP_LED_Init(LED1);   /* heartbeat in ControlTask */

    /* -------- Motor PWM (TIM8 right, TIM4 left) -------- */
    uwPrescalerValue = (SystemCoreClock / 2) / 1000000;

    __GPIOB_CLK_ENABLE();
    GPIO_InitStruct.Pin   = GPIO_PIN_2;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);   /* MC_EN */

    sConfig1.OCMode     = TIM_OCMODE_PWM1;
    sConfig1.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfig1.OCFastMode = TIM_OCFAST_DISABLE;
    sConfig1.Pulse      = 0;                    /* CCR set by Motor_Drive at runtime */

    TimHandle1.Instance               = TIM8;
    TimHandle1.Init.Prescaler         = uwPrescalerValue;
    TimHandle1.Init.Period            = PWM_PERIOD;
    TimHandle1.Init.ClockDivision     = 0;
    TimHandle1.Init.CounterMode       = TIM_COUNTERMODE_UP;
    HAL_TIM_PWM_Init(&TimHandle1);
    HAL_TIM_PWM_ConfigChannel(&TimHandle1, &sConfig1, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&TimHandle1, &sConfig1, TIM_CHANNEL_2);

    sConfig2 = sConfig1;
    TimHandle2.Instance               = TIM4;
    TimHandle2.Init.Prescaler         = uwPrescalerValue;
    TimHandle2.Init.Period            = PWM_PERIOD;
    TimHandle2.Init.ClockDivision     = 0;
    TimHandle2.Init.CounterMode       = TIM_COUNTERMODE_UP;
    HAL_TIM_PWM_Init(&TimHandle2);
    HAL_TIM_PWM_ConfigChannel(&TimHandle2, &sConfig2, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&TimHandle2, &sConfig2, TIM_CHANNEL_2);

    /* Start all 4 motor PWM channels once; Motor_Drive only updates CCR */
    HAL_TIM_PWM_Start(&TimHandle1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&TimHandle1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&TimHandle2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&TimHandle2, TIM_CHANNEL_2);
    TIM8->CCR1 = 0; TIM8->CCR2 = 0;
    TIM4->CCR1 = 0; TIM4->CCR2 = 0;

    EXTILine_Config();

    /* -------- Ultrasonic input capture (TIM3 CH2/3/4) -------- */
    uwPrescalerValue = ((SystemCoreClock / 2) / 1000000) - 1;
    TimHandle3.Instance               = TIM3;
    TimHandle3.Init.Period            = 0xFFFF;
    TimHandle3.Init.Prescaler         = uwPrescalerValue;
    TimHandle3.Init.ClockDivision     = 0;
    TimHandle3.Init.CounterMode       = TIM_COUNTERMODE_UP;
    if (HAL_TIM_IC_Init(&TimHandle3) != HAL_OK) Error_Handler();

    sICConfig.ICPolarity  = TIM_ICPOLARITY_RISING;
    sICConfig.ICSelection = TIM_ICSELECTION_DIRECTTI;
    sICConfig.ICPrescaler = TIM_ICPSC_DIV1;
    sICConfig.ICFilter    = 0;
    HAL_TIM_IC_ConfigChannel(&TimHandle3, &sICConfig, TIM_CHANNEL_1);
    HAL_TIM_IC_ConfigChannel(&TimHandle3, &sICConfig, TIM_CHANNEL_2);
    HAL_TIM_IC_ConfigChannel(&TimHandle3, &sICConfig, TIM_CHANNEL_3);
    HAL_TIM_IC_ConfigChannel(&TimHandle3, &sICConfig, TIM_CHANNEL_4);
    HAL_TIM_IC_Start_IT(&TimHandle3, TIM_CHANNEL_2);
    HAL_TIM_IC_Start_IT(&TimHandle3, TIM_CHANNEL_3);
    HAL_TIM_IC_Start_IT(&TimHandle3, TIM_CHANNEL_4);

    /* -------- Ultrasonic trigger (TIM10 CH1) -------- */
    uwPrescalerValue = (SystemCoreClock / 2 / 131099) - 1;
    TimHandle4.Instance               = TIM10;
    TimHandle4.Init.Prescaler         = uwPrescalerValue;
    TimHandle4.Init.Period            = 0xFFFF;
    TimHandle4.Init.ClockDivision     = 0;
    TimHandle4.Init.CounterMode       = TIM_COUNTERMODE_UP;
    HAL_TIM_PWM_Init(&TimHandle4);

    sConfig3.OCMode     = TIM_OCMODE_PWM1;
    sConfig3.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfig3.OCFastMode = TIM_OCFAST_DISABLE;
    sConfig3.Pulse      = TRIG_PULSE;
    HAL_TIM_PWM_ConfigChannel(&TimHandle4, &sConfig3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&TimHandle4, TIM_CHANNEL_1);

    /* -------- IR ADC1/2/3 -------- */
    AdcHandle1.Instance                   = ADC3;
    AdcHandle1.Init.ClockPrescaler        = ADC_CLOCKPRESCALER_PCLK_DIV2;
    AdcHandle1.Init.Resolution            = ADC_RESOLUTION12b;
    AdcHandle1.Init.ScanConvMode          = DISABLE;
    AdcHandle1.Init.ContinuousConvMode    = DISABLE;
    AdcHandle1.Init.DiscontinuousConvMode = DISABLE;
    AdcHandle1.Init.NbrOfDiscConversion   = 0;
    AdcHandle1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    AdcHandle1.Init.ExternalTrigConv      = ADC_EXTERNALTRIGCONV_T1_CC1;
    AdcHandle1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    AdcHandle1.Init.NbrOfConversion       = 1;
    AdcHandle1.Init.DMAContinuousRequests = DISABLE;
    AdcHandle1.Init.EOCSelection          = DISABLE;
    HAL_ADC_Init(&AdcHandle1);
    adcConfig1.Channel      = ADC_CHANNEL_11;
    adcConfig1.Rank         = 1;
    adcConfig1.SamplingTime = ADC_SAMPLETIME_480CYCLES;
    adcConfig1.Offset       = 0;
    HAL_ADC_ConfigChannel(&AdcHandle1, &adcConfig1);

    AdcHandle2.Instance                   = ADC2;
    AdcHandle2.Init.ClockPrescaler        = ADC_CLOCKPRESCALER_PCLK_DIV2;
    AdcHandle2.Init.Resolution            = ADC_RESOLUTION12b;
    AdcHandle2.Init.ScanConvMode          = DISABLE;
    AdcHandle2.Init.ContinuousConvMode    = DISABLE;
    AdcHandle2.Init.DiscontinuousConvMode = DISABLE;
    AdcHandle2.Init.NbrOfDiscConversion   = 0;
    AdcHandle2.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    AdcHandle2.Init.ExternalTrigConv      = ADC_EXTERNALTRIGCONV_T1_CC1;
    AdcHandle2.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    AdcHandle2.Init.NbrOfConversion       = 1;
    AdcHandle2.Init.DMAContinuousRequests = DISABLE;
    AdcHandle2.Init.EOCSelection          = DISABLE;
    HAL_ADC_Init(&AdcHandle2);
    adcConfig2.Channel      = ADC_CHANNEL_14;
    adcConfig2.Rank         = 1;
    adcConfig2.SamplingTime = ADC_SAMPLETIME_480CYCLES;
    adcConfig2.Offset       = 0;
    HAL_ADC_ConfigChannel(&AdcHandle2, &adcConfig2);

    AdcHandle3.Instance                   = ADC1;
    AdcHandle3.Init.ClockPrescaler        = ADC_CLOCKPRESCALER_PCLK_DIV2;
    AdcHandle3.Init.Resolution            = ADC_RESOLUTION12b;
    AdcHandle3.Init.ScanConvMode          = DISABLE;
    AdcHandle3.Init.ContinuousConvMode    = DISABLE;
    AdcHandle3.Init.DiscontinuousConvMode = DISABLE;
    AdcHandle3.Init.NbrOfDiscConversion   = 0;
    AdcHandle3.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    AdcHandle3.Init.ExternalTrigConv      = ADC_EXTERNALTRIGCONV_T1_CC1;
    AdcHandle3.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    AdcHandle3.Init.NbrOfConversion       = 1;
    AdcHandle3.Init.DMAContinuousRequests = DISABLE;
    AdcHandle3.Init.EOCSelection          = DISABLE;
    HAL_ADC_Init(&AdcHandle3);
    adcConfig3.Channel      = ADC_CHANNEL_15;
    adcConfig3.Rank         = 1;
    adcConfig3.SamplingTime = ADC_SAMPLETIME_480CYCLES;
    adcConfig3.Offset       = 0;
    HAL_ADC_ConfigChannel(&AdcHandle3, &adcConfig3);

    /* -------- RTOS tasks (sizes match main_good) -------- */
    BSP_LED_Init(LED2);
    BSP_LED_On(LED1);                /* solid ON pre-scheduler -> blinking = ControlTask alive */
    xTaskCreate(SensorTask,  "sensor",   512, NULL, 3, NULL);
    xTaskCreate(IR_Task,     "ir",       512, NULL, 3, NULL);
    xTaskCreate(ControlTask, "control", 1024, NULL, 2, NULL);
    xTaskCreate(DebugTask,   "debug",    512, NULL, 1, NULL);
    vTaskStartScheduler();

    /* Only reached if scheduler failed -> LED2 solid ON as fault marker */
    BSP_LED_On(LED2);
    while (1) { }
}

/* ===========================================================================
 *  System clock
 * =========================================================================== */
/**
 * @brief  Configures HSE+PLL clock tree to 180MHz with overdrive.
 * @note   PLL: HSE / 25 * 360 / 2 = 180MHz SYSCLK. AHB=180, APB1=45, APB2=90.
 *         Flash latency set to 5 wait states for 180MHz operation.
 */
static void SystemClock_Config(void)
{
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_OscInitTypeDef RCC_OscInitStruct;

    __PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM       = 25;
    RCC_OscInitStruct.PLL.PLLN       = 360;
    RCC_OscInitStruct.PLL.PLLP       = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ       = 7;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);
    HAL_PWREx_ActivateOverDrive();

    RCC_ClkInitStruct.ClockType      = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                                        RCC_CLOCKTYPE_PCLK1  | RCC_CLOCKTYPE_PCLK2);
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  HAL assert hook used when USE_FULL_ASSERT is defined.
 * @param  file  Source file where the assert tripped (unused).
 * @param  line  Line number of the failing assert (unused).
 * @note   Currently traps in an infinite loop without logging.
 */
void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file; (void)line;
    while (1) { }
}
#endif

/* ===========================================================================
 *  ISR callbacks
 * =========================================================================== */
/**
 * @brief  EXTI ISR — increments encoder counters from quadrature edges.
 * @param  GPIO_Pin  Pin that triggered the EXTI (PA15 = right, PB4 = left).
 * @note   Reads the partner pin (PB3 / PB5) to decide direction. Counter
 *         increments on logic-0 partner, decrements on logic-1. `READY` is
 *         the sentinel between edges so transient reads don't double-count.
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    switch (GPIO_Pin) {
        case GPIO_PIN_15:
            encoder_right = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3);
            if      (encoder_right == 0) { motorInterrupt1++; encoder_right = READY; }
            else if (encoder_right == 1) { motorInterrupt1--; encoder_right = READY; }
            break;
        case GPIO_PIN_4:
            encoder_left = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5);
            if      (encoder_left == 0) { motorInterrupt2++; encoder_left = READY; }
            else if (encoder_left == 1) { motorInterrupt2--; encoder_left = READY; }
            break;
    }
}

/**
 * @brief  TIM input-capture ISR — measures HC-SR04 echo pulse widths on TIM3.
 * @param  htim  HAL timer handle; only TIM3 is handled here.
 * @note   Each of CH2 (right), CH3 (front), CH4 (left) toggles capture
 *         polarity on every edge. Pulse width = falling - rising, with
 *         16-bit wraparound handled inline. Result feeds uwDiffCapture1/2/3
 *         consumed by SensorTask (divided by US_TICKS_PER_CM for cm).
 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM3) return;

    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
        if ((TIM3->CCER & TIM_CCER_CC2P) == 0) {
            uwIC2Value1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
            TIM3->CCER |= TIM_CCER_CC2P;
        } else {
            uwIC2Value2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
            uwDiffCapture1 = (uwIC2Value2 > uwIC2Value1)
                ? (uwIC2Value2 - uwIC2Value1)
                : ((0xFFFF - uwIC2Value1) + uwIC2Value2);
            TIM3->CCER &= ~TIM_CCER_CC2P;
        }
    }
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
        if ((TIM3->CCER & TIM_CCER_CC3P) == 0) {
            uwIC2Value3 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
            TIM3->CCER |= TIM_CCER_CC3P;
        } else {
            uwIC2Value4 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
            uwDiffCapture2 = (uwIC2Value4 > uwIC2Value3)
                ? (uwIC2Value4 - uwIC2Value3)
                : ((0xFFFF - uwIC2Value3) + uwIC2Value4);
            TIM3->CCER &= ~TIM_CCER_CC3P;
        }
    }
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) {
        if ((TIM3->CCER & TIM_CCER_CC4P) == 0) {
            uwIC2Value5 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
            TIM3->CCER |= TIM_CCER_CC4P;
        } else {
            uwIC2Value6 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
            uwDiffCapture3 = (uwIC2Value6 > uwIC2Value5)
                ? (uwIC2Value6 - uwIC2Value5)
                : ((0xFFFF - uwIC2Value5) + uwIC2Value6);
            TIM3->CCER &= ~TIM_CCER_CC4P;
        }
    }
}

/**
 * @brief  Fatal-error trap — lights LED3 and stops forward progress.
 * @note   Called when an HAL init routine returns HAL_ERROR. No recovery.
 */
static void Error_Handler(void)
{
    BSP_LED_On(LED3);
    while (1) { }
}

/**
 * @brief  Wires up the encoder EXTI lines (PA15, PB3/4/5) and enables NVIC.
 * @note   PA15 -> EXTI15_10 (right encoder edge). PB4 -> EXTI4 (left encoder
 *         edge). PB3/PB5 are partner pins read inside the EXTI callback to
 *         determine rotation direction. All set to rising-edge interrupts.
 */
static void EXTILine_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    __GPIOA_CLK_ENABLE();
    GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    GPIO_InitStructure.Pin  = GPIO_PIN_15;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

    __GPIOB_CLK_ENABLE();
    GPIO_InitStructure.Pin = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
    HAL_NVIC_SetPriority(EXTI4_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(EXTI4_IRQn);
}

/************************ (C) BaseLine ************************/