/*
 * M2003 Hardware Configuration (Pin-Compatible with MS51 Board)
 *
 * This header maps the physical pins of the BL4818 board to M2003 peripherals.
 */

#ifndef M2003_CONFIG_H
#define M2003_CONFIG_H

/* CPU Clock */
#define CLK_HIRC_24M        24000000UL

/* ── PWM Gate Drives (EPWM0) ────────────────────────────────────────────── */
/* EPWM0 channels 0-5 map to the 6 MOSFET gates. */
/* M2003 TSSOP20 pin mapping: */
#define PIN_PWM_U_LOW       PB13      /* PB.13, Pin 13, EPWM0_CH0 */
#define PIN_PWM_U_HIGH      PB12      /* PB.12, Pin 14, EPWM0_CH1 */
#define PIN_PWM_V_HIGH      PB7       /* PB.7, Pin 15, EPWM0_CH2 */
#define PIN_PWM_V_LOW       PB8      /* PB.8, Pin 16, EPWM0_CH3 */
#define PIN_PWM_W_LOW       PB9      /* PB.9, Pin 17, EPWM0_CH4 */
#define PIN_PWM_W_HIGH      PB11      /* PB.11, Pin 19, EPWM0_CH5 */

/* PWM Scaling */
#define PWM_FREQ_HZ         20000
#define PWM_PERIOD          (CLK_HIRC_24M / PWM_FREQ_HZ)
#define PWM_MAX_DUTY        1200    /* 24MHz / 20kHz */

/* ── UART Ring (UART1) ─────────────────────────────────────────────────── */
#define PIN_UART1_TX        PF0     /* Pin 8  */
#define PIN_UART1_RX        PF1     /* Pin 18 */

/* ── ADC Sensing ───────────────────────────────────────────────────────── */
#define ADC_CH_CURRENT      2       /* PB.2, Pin 2 */
#define ADC_CH_VOLTAGE      3       /* PB.3, Pin 3 */

/* ── SSI Encoder (Bit-Banged) ──────────────────────────────────────────── */
#define PIN_SSI_CLK         PB0       /* PB.0, Pin 20 (direct drive) */
#define PIN_SSI_CSN         PB1       /* PB.1, Pin 1 (inverted via N-FET) */
#define PIN_SSI_DAT         PB15      /* PB.15, Pin 11 */

/* ── Commutation ──────────────────────────────────────────────────────── */
#define COMMUTATION_OFFSET  1       /* Advance commutation by 1 sector (matches MS51 tuning) */
#define HALL_MIN_TRANSITION_US 20   /* reject implausibly fast hall chatter / edge bounce */

/* ── Motor Parameters ─────────────────────────────────────────────────── */
#define MOTOR_POLE_PAIRS    5       /* 10-pole, 12 slot outrunner */
#define HALL_TRANSITIONS_PER_REV (6 * MOTOR_POLE_PAIRS)  /* 30 */
#define ENCODER_COUNTS_PER_REV  16384   /* 14-bit absolute encoder */

/* ── Loop Rates ──────────────────────────────────────────────────────── */
#define CURRENT_LOOP_HZ     5000   /* fastest software loop: ADC/protection/current PI */
#define VELOCITY_LOOP_HZ    2500    /* velocity estimator + velocity PID               */
#define POSITION_LOOP_HZ    1250    /* position PID (outermost)                        */
#define STRIKE_LOOP_HZ      1000    /* strike state machine                            */
#define PROTOCOL_TICK_HZ    1000    /* serial parser timeout aging                     */
#define PROTOCOL_FRAME_TIMEOUT_MS 20u
#define ENCODER_POLL_HZ     VELOCITY_LOOP_HZ /* bit-banged SSI read rate            */
#define CONTROL_LOOP_HZ     CURRENT_LOOP_HZ  /* SysTick fast tick budget/reporting   */

/* ── Timing Helpers ──────────────────────────────────────────────────── */
#define HZ_TICKS_FROM_MS(hz, ms) \
    ((uint32_t)((((uint64_t)(hz) * (uint64_t)(ms)) + 999ULL) / 1000ULL))
#define HZ_TICKS_FROM_US(hz, us) \
    ((uint32_t)((((uint64_t)(hz) * (uint64_t)(us)) + 999999ULL) / 1000000ULL))
#define VALUE_PER_TICK_FROM_HZ(hz, per_second) \
    ((uint32_t)((((uint64_t)(per_second)) + (uint64_t)(hz) - 1ULL) / (uint64_t)(hz)))

#if VELOCITY_LOOP_HZ > CONTROL_LOOP_HZ
#error "VELOCITY_LOOP_HZ must be <= CONTROL_LOOP_HZ"
#endif

#if POSITION_LOOP_HZ > CONTROL_LOOP_HZ
#error "POSITION_LOOP_HZ must be <= CONTROL_LOOP_HZ"
#endif

#if STRIKE_LOOP_HZ > CONTROL_LOOP_HZ
#error "STRIKE_LOOP_HZ must be <= CONTROL_LOOP_HZ"
#endif

#if PROTOCOL_TICK_HZ > CONTROL_LOOP_HZ
#error "PROTOCOL_TICK_HZ must be <= CONTROL_LOOP_HZ"
#endif

#if ENCODER_POLL_HZ != VELOCITY_LOOP_HZ
#error "Bit-banged SSI encoder polling is expected to match VELOCITY_LOOP_HZ"
#endif

/* ── ADC / Current Sense ─────────────────────────────────────────────── */
#define ADC_FULL_SCALE_COUNTS        4095UL
#define ADC_VREF_MV                  5000UL
#define CURRENT_SENSE_SHUNT_MOHM     20UL
#define CURRENT_SENSE_GAIN           50UL

/* ── Current Limits ──────────────────────────────────────────────────── */
#define DEFAULT_TORQUE_LIMIT_MA 3200
#define CURRENT_LIMIT_MA        3800    /* sustained limit — ADC full scale ~4900 mA (5 V AVDD, INA180B2 on 5 V) */
#define CURRENT_PEAK_LIMIT_MA   4500    /* near-instant fault ceiling for real stalls / shoot-through */
#define CURRENT_PEAK_FAULT_TIME_US 300  /* require this much peak-overlimit time before faulting */
#define CURRENT_FILTER_SHIFT    3       /* IIR alpha = 1/(1<<N) for reported/clamped current */
#define OVERCURRENT_FAULT_TIME_MS 8     /* sustained over-limit time before faulting */
#define CURRENT_PEAK_FAULT_SAMPLES HZ_TICKS_FROM_US(PWM_FREQ_HZ, CURRENT_PEAK_FAULT_TIME_US)
#define OVERCURRENT_FAULT_SAMPLES HZ_TICKS_FROM_MS(PWM_FREQ_HZ, OVERCURRENT_FAULT_TIME_MS)

/* Current PI defaults (Q8 — inner loop, regulates motor current to setpoint) */
#define CUR_PID_KP_DEFAULT  96       /* 0.25 duty/mA */
#define CUR_PID_KI_DEFAULT  5        /* 0.03125 */

/*this is used in the D term of both position and velocity loops*/
#define PID_D_FILTER_SHIFT  2       /* D-term IIR alpha = 1/(1<<N): 2→1/4, 3→1/8 */

/* Velocity PID defaults (Q8 — output is current command in mA, not duty) */
#define VEL_PID_KP_DEFAULT  1536     /* 15.0 mA/RPM */
#define VEL_PID_KI_DEFAULT  10       /* 0.29 */
#define VEL_PID_KD_DEFAULT  20       /* 0.29 */
#define VEL_FF_DEFAULT      220      /* feedforward: mA per RPM */
#define VEL_FILTER_SHIFT    3       /* IIR alpha = 1/(1<<N): 2→1/4, 3→1/8 */

/* Position PID defaults (Q8 — output is velocity RPM)
 * Error is prescaled by POS_ERROR_PRESCALE for finer gain resolution.
 * Effective gain = Q8_value / 256 / POS_ERROR_PRESCALE RPM per encoder count */
#define POS_ERROR_PRESCALE  16      /* divide position error by 16 before PID */
#define POS_PID_KP_DEFAULT  1200     /* ≈0.0625 RPM/count (same as old 16 w/o prescale) */
#define POS_PID_KI_DEFAULT  10
#define POS_PID_KD_DEFAULT  20
#define POS_MAX_VEL_RPM     2000    /* position loop velocity clamp */
#define POS_INT_MAX_RPM     4000    /* position integral velocity clamp (RPM) */

/* ── Encoder Health ──────────────────────────────────────────────────── */
/* If motor is being driven with meaningful duty for this long without any
 * encoder position change, assume the SSI bus has died and fault out.
 * Below ENCODER_STALL_MIN_DUTY we allow stasis (hold-at-target is normal). */
#define ENCODER_STALL_TIME_MS        1000u
#define ENCODER_STALL_MIN_DUTY       (PWM_MAX_DUTY / 8)    /* 12.5% — real drive, not a rest-hold */
#define ENCODER_STALL_FAULT_SAMPLES  HZ_TICKS_FROM_MS(PWM_FREQ_HZ, ENCODER_STALL_TIME_MS)

/* ── ADC Health ───────────────────────────────────────────────────────── */
/* ADC is hardware-triggered at PWM rate (20 kHz), so every fast tick
 * should normally see several samples.  If we see zero samples for this
 * many consecutive fast ticks, the ADC/PWM trigger path has failed. */
#define ADC_SILENCE_FAULT_TICKS      20u    /* 4 ms at CURRENT_LOOP_HZ = 5 kHz */

/* ── Strike Defaults ──────────────────────────────────────────────────── */
#define STRIKE_HOME_OFFSET_DEFAULT      1024    /* encoder counts above drum surface */
#define STRIKE_COAST_DISTANCE_DEFAULT   300     /* cut power this far from drum (counts) */
#define STRIKE_HOMING_DUTY_DEFAULT      100  /* low duty toward drum (sign = toward drum) */
#define STRIKE_MAX_REBOUND_OVERSHOOT    512     /* counts beyond home allowed before forced recapture */
#define STRIKE_RETRIGGER_DEADBAND       512     /* counts: must be this close to home for predictable retrigger */
#define STRIKE_RETRIGGER_VEL_THRESHOLD  512     /* RPM: must be this slow near home for predictable retrigger */
#define STRIKE_SETTLE_DEADBAND          200      /* counts: position "settled" threshold */
#define STRIKE_SETTLE_TIME_MS           20
#define STRIKE_HOMING_STALL_TIME_MS     200
#define STRIKE_HOMING_STALL_THRESHOLD   4       /* counts: less than this = stalled */
#define STRIKE_COAST_TIMEOUT_MS         500
#define STRIKE_REBOUND_THRESHOLD        5       /* RPM away from drum to confirm rebound */
#define STRIKE_SETTLE_TICKS HZ_TICKS_FROM_MS(STRIKE_LOOP_HZ, STRIKE_SETTLE_TIME_MS)
#define STRIKE_HOMING_STALL_TICKS HZ_TICKS_FROM_MS(STRIKE_LOOP_HZ, STRIKE_HOMING_STALL_TIME_MS)
#define STRIKE_COAST_TIMEOUT_TICKS HZ_TICKS_FROM_MS(STRIKE_LOOP_HZ, STRIKE_COAST_TIMEOUT_MS)

#endif /* M2003_CONFIG_H */
