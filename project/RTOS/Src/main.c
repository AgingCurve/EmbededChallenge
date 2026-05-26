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
#define TASK_WARMUP_MS    200
#define CTRL_WARMUP_MS    300

/* Distance thresholds (cm) */
#define D_TARGET          15       /* wall-follow target distance */
#define D_MIN             8        /* lower safety bound          */
#define EMG_FRONT         20        /* emergency front threshold   */
#define EMG_FRONT_HYST    2        /* +cm margin to clear EMERGENCY */

/* Motor PWM */
#define PWM_PERIOD        20000
#define V_CRUISE          20000    /* duty for SEEK forward drive */
#define V_TURN            18000    /* duty for in-place pivot (each wheel) */

/* HC-SR04 trigger */
#define TRIG_PULSE        2

/* ADC */
#define ADC_POLL_TIMEOUT  0xFF

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

int ir_floor = 0, ir_left = 0, ir_right = 0;

DriveState   state = START;
TrackingSide side  = TRACK_RIGHT;

/* ===========================================================================
 *  Forward declarations
 * =========================================================================== */
static void SystemClock_Config(void);
static void EXTILine_Config(void);
static void Error_Handler(void);

/* Sensing */
int  median7(int *a);
int  stddev7(int *a);
void SensorTask(void *arg);
void IR_Task(void *arg);

/* Control */
void    Motor_Drive(int v_left, int v_right);
void    Motor_Stop(void);
void    switchTracking(void);
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

        osDelay(SENS_PERIOD_MS);
    }
}

/* ===========================================================================
 *  Sensing Layer — IR_Task
 * =========================================================================== */
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
    }
}

/* ===========================================================================
 *  Motor Layer
 *  v_left / v_right ∈ [-PWM_PERIOD, +PWM_PERIOD].  Sign = direction.
 *  TIM8 drives right motor (CH1=fwd, CH2=rev); TIM4 drives left (CH2=fwd, CH1=rev).
 * =========================================================================== */
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

void Motor_Stop(void)
{
    /* Keep PWM channels running; just zero the duty so motor coasts.
     * (HAL_TIM_PWM_Stop would disable the channel and subsequent
     *  Motor_Drive(CCR=..) writes would produce no output.) */
    Motor_Drive(0, 0);
}

/* ===========================================================================
 *  Control Layer — helpers
 * =========================================================================== */
void switchTracking(void)
{
    /* TODO: flip `side` based on which wall is closer / more stable.
     *       Compare (dL, sL) vs (dR, sR); prefer the lower-stddev side. */
}

void angleAdjusting(void)
{
    /* TODO: small differential nudge to drive angleCalculate() toward 0.
     *       Use Motor_Drive() with asymmetric v_left/v_right. */
}

int angleCalculate(void)
{
    /* TODO: estimate yaw error from rate-of-change on the tracked side.
     *       Positive = drifting away from wall, negative = drifting in. */
    return 0;
}

uint8_t canProgressDirection(void)
{
    /* TODO: return bitmask of DIR_LEFT | DIR_FORWARD | DIR_RIGHT. */
    return 0;
}

bool isEmergency(void)
{
    /* Floor IR not wired on this build -> rely on front ultrasonic only. */
    return (dF > 0 && dF <= EMG_FRONT);
}

bool emergencyResolved(void)
{
    return (dF > EMG_FRONT + EMG_FRONT_HYST);
}

/* ===========================================================================
 *  Control Layer — FSM
 * =========================================================================== */
void ControlTask(void *arg)
{
    (void)arg;
    static const char *state_name[] = {
        "START","SEEK","ALIGNED","LOCKED","EMERGENCY","INTERSECT","ENCOUNT","STOP"
    };
    uint32_t tick = 0;

    osDelay(CTRL_WARMUP_MS);
    for (;;) {
        if (isEmergency()) state = EMERGENCY;

        /* LED1 toggles every ~500ms -> visual proof scheduler is alive */
        if ((tick++ % 25) == 0) {
            BSP_LED_Toggle(LED1);
            printf("\r\n[%s] dF=%d dL=%d dR=%d | sF=%d sL=%d sR=%d | irF=%d irL=%d irR=%d",
                   state_name[state], dF, dL, dR, sF, sL, sR, ir_floor, ir_left, ir_right);
        }

        switch (state) {
            case START:
                /* TODO: wait until sensors warm up; transition to SEEK. */
                state = SEEK;
                break;

            case SEEK: {
                /* Drive forward; veer away from whichever side gets too close. */
                int vL = V_CRUISE, vR = V_CRUISE;
                if (dR > 0 && dR < D_MIN) vR = V_CRUISE / 2;   /* right wall close -> curve left */
                if (dL > 0 && dL < D_MIN) vL = V_CRUISE / 2;   /* left  wall close -> curve right */
                Motor_Drive(vL, vR);
            } break;

            case ALIGNED:
                /* TODO: cruise. angleAdjusting() each tick; watch dF for INTERSECT. */
                break;

            case LOCKED:
                /* TODO: all sides under D_MIN — back off and re-seek. */
                break;

            case EMERGENCY:
                /* Front blocked: pivot in place toward the more open side.
                 * dL >= dR -> left is more open -> pivot left  (CCW: -L, +R)
                 * dL <  dR -> right is more open -> pivot right (CW : +L, -R) */
                if (dL >= dR) Motor_Drive(-V_TURN,  V_TURN);
                else          Motor_Drive( V_TURN, -V_TURN);
                if (emergencyResolved()) state = SEEK;
                break;

            case INTERSECT:
                /* TODO: use canProgressDirection() to pick a turn. */
                break;

            case ENCOUNT:
                /* TODO: brief pause; if obstacle persists, treat as wall. */
                break;

            case STOP:
                Motor_Stop();
                break;
        }
        osDelay(CTRL_PERIOD_MS);
    }
}

/* ===========================================================================
 *  main
 * =========================================================================== */
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
    vTaskStartScheduler();

    /* Only reached if scheduler failed -> LED2 solid ON as fault marker */
    BSP_LED_On(LED2);
    while (1) { }
}

/* ===========================================================================
 *  System clock
 * =========================================================================== */
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
void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file; (void)line;
    while (1) { }
}
#endif

/* ===========================================================================
 *  ISR callbacks
 * =========================================================================== */
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

static void Error_Handler(void)
{
    BSP_LED_On(LED3);
    while (1) { }
}

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
