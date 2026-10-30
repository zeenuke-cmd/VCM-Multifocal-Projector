/*
 * VCM_Unified.c -- calibration + 60 Hz flip in ONE ATmega128 firmware
 * Target: ATmega128 / ATmega128A, 16 MHz, avr-gcc, single source file.
 *
 * Sources merged without retuning:
 *   CAL  : VCM_Control_Updated.c         (range M, single target T<n>, ST)
 *   FLIP : Adaptive_Base_V3_Fast_Hold.c  (T<front>,<back>, GPIO17 sync)
 *
 * Wiring and polarity unchanged:
 *   Hall PF1/ADC1, PWM PB5/OC1A (40 kHz, TOP 399), DIR PB0 LOW, nSLEEP PB7.
 *   ST  : PD4 -> level shifter -> Pi GPIO22   (CAL mode only)
 *   SYNC: Pi GPIO17 -> PE4                    (FLIP mode only)
 *   UART1: PD2 RX / PD3 TX 115200 (Pi commands). UART0: PE1 TX log.
 *   More PWM -> lower Hall ADC. No reverse drive.
 *
 * ------------------------------------------------------------------
 * Timing (the key point of the merge)
 * ------------------------------------------------------------------
 *   Timer0 CTC, clk/32 (ATmega128 Timer0 prescaler table), OCR0=249
 *   -> 2 kHz tick (0.5 ms). Every tick: one ADC conversion + PE4 sample.
 *
 *   FLIP controller runs EVERY tick   (2 kHz)  = Fast_Hold as validated.
 *   CAL  controller runs EVERY 2nd tick (1 kHz) = VCM_Control_Updated as
 *        validated: 1 ms PI-D units, +/-70 dither at 500 Hz, median3 +
 *        pair filter, 150 ms settle, 2.5 s range dwell -- all unchanged.
 *
 *   Both filters are updated in every mode, so switching modes starts
 *   from the real current position (no stale filter state).
 *
 * ------------------------------------------------------------------
 * UART1 commands (existing Pi programs need NO change)
 * ------------------------------------------------------------------
 *   T<n>            CAL: single target (division20 heartbeat, ST on PD4)
 *   T<a>,<b>  a!=b  FLIP. ORDER DOES NOT MATTER:
 *                   GPIO17 HIGH -> the larger value  (Mango shows the HIGH-ADC image)
 *                   GPIO17 LOW  -> the smaller value (Mango shows the LOW-ADC image)
 *                   so T<high>,<low> and T<low>,<high> behave the same.
 *   T<a>,<a>        same value twice (Mango contrast hold) -> CAL T<a>
 *   M               CAL range measure -> LIMIT:min:max / RANGE:min:max
 *   X               stop everything, motor off  -> STOPPED
 *   Q               CAL: POS:adc:st    FLIP: TARGETS:<low>:<high>:valid
 *   S               SYNC:<level>:<edge_count>
 *   R               CAL bumpless reset (ERR:CMD in FLIP)
 *   P/I/D/G/A<n>    CAL gains / feedforward / dither (unchanged)
 *
 *   Same T (single or pair) repeated = heartbeat, never resets learning.
 *
 * ------------------------------------------------------------------
 * FLIP sync watchdog (new, ATmega-local)
 * ------------------------------------------------------------------
 *   Mango does not send X when the flip ends; GPIO17 just stops toggling.
 *   If PE4 does not change for FLIP_SYNC_TIMEOUT_TICKS (200 ms), the
 *   motor is turned off. Targets are kept; control resumes on the next
 *   edge. Right after T<a>,<b> the motor also waits for the first edge.
 *
 * ------------------------------------------------------------------
 * UART0 log: format follows the active mode (existing parsers work)
 * ------------------------------------------------------------------
 *   CAL/RANGE/STOP : "Target:.. | ADC:.. | ... | t:.." every 25 ms
 *   FLIP           : "F,adc,err,P,I,D,PWM,TICK" every line (as Fast_Hold)
 *
 * Changes vs. the two sources (only what the merge required):
 *   - CAL ADC read moved from ADC interrupt to the shared timer tick
 *     (same sample instant, 1 kHz as before). AdcBusy stays 0.
 *   - FLIP PWM=0 now truly LOW (CAL's set_pwm_raw), was a narrow pulse.
 *   - Miss threshold scaled to the 0.5 ms tick (>0.625 ms).
 *   - ExecUs is the whole Timer0 ISR (ADC + both filters + controller).
 *
 * Build: avr-gcc -mmcu=atmega128 -DF_CPU=16000000UL -Os -std=gnu99 \
 *        -Wall -Wextra VCM_Unified.c -o vcm.elf
 */

#ifndef F_CPU
#define F_CPU 16000000UL
#endif
#if F_CPU != 16000000UL
#error This configuration is for a 16 MHz ATmega128/128A.
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <stdint.h>

/* ================================================================
 * Pins / common
 * ================================================================ */
#define HALL_CH                 1U
#define DIR_PIN                 PB0
#define SLEEP_PIN               PB7
#define PWM_PIN                 PB5
#define ST_PIN                  PD4
#define SYNC_PIN                PE4
#define BAUD                    115200UL
#define UBRR_VALUE              ((F_CPU + 4UL * BAUD) / (8UL * BAUD) - 1UL)
#define PWM_TOP                 399U
#define PWM_MIN                 0
#define PWM_MAX                 350

#define TICK_HZ                 2000UL
#define TIMER0_OCR              249U        /* 16e6 / 32 / 250 = 2000 Hz */
#define TIMER3_COUNTS_PER_TICK  1000U       /* Timer3 /8 = 0.5 us */
#define MISS_THRESHOLD_COUNTS   1250U       /* > 0.625 ms */

#define RX_SIZE                 128U
#define RX_MASK                 (RX_SIZE - 1U)
#define CMD_SIZE                24U

#define MODE_PID                0U          /* CAL single target */
#define MODE_RANGE              1U          /* CAL M measurement  */
#define MODE_STOP               2U
#define MODE_FLIP               3U

/* ================================================================
 * CAL constants (VCM_Control_Updated.c, unchanged, units = 1 ms)
 * ================================================================ */
#define PWM_START               175
#define Q8                      256L
#define CONTROL_SCALE           128L
#define I_CONTROL_SCALE         4096L
#define KP_DEFAULT              128U
#define KI_DEFAULT              48U
#define KD_DEFAULT              1536U
#define KP_MAX                  1024U
#define KI_MAX                  256U
#define KD_MAX                  8192U
#define I_ERROR_LIMIT_Q8        (12L * Q8)
#define I_LIMIT_Q8              (350L * Q8)
#define REF_STEP_Q8             128L    /* 0.5 ADC per 1 ms */
#define PWM_SLEW_Q8             (4L * Q8)
#define PWM_BRAKE_SLEW_Q8       (30L * Q8)
#define BRAKE_SPEED_Q8          64L     /* 0.25 ADC/ms */
#define POSITION_TOL_Q8         (2L * Q8)
#define STABLE_EXIT_TOL_Q8      (5L * Q8)
#define STABLE_SPEED_Q8         32L     /* 0.125 ADC/ms */
#define STABLE_MS               150U
#define SAT_TIMEOUT_MS          2000U
#define DITHER_DEFAULT          70U
#define DITHER_MAX              70U
#define FF_MAX_PWM              8L
#define GAIN_DEFAULT            154U
#define GAIN_MIN                32U
#define GAIN_MAX                2560U
#define RANGE_SETTLE_MS         2500U
#define RANGE_SAMPLES           150U
#define RANGE_RETURN_MS         500U
#define REACH_MIN_SPAN          24
#define EDGE_MARGIN             10
#define DEFAULT_REACH_MIN       100
#define DEFAULT_REACH_MAX       399
#define SENSOR_RAIL_MS          20U
#define PRINT_PERIOD_MS         25U

/* ================================================================
 * FLIP constants (Adaptive_Base_V3_Fast_Hold.c, unchanged, 0.5 ms tick)
 * ================================================================ */
#define TARGET_ADC_MIN          100U
#define TARGET_ADC_MAX          1023U
#define BASE_REF_ADC            300
#define BASE_REF_PWM            80
#define BASE_PWM_PER_ADC        2
#define F_PWM_MAX               PWM_MAX

#define ADC_SCALE               16L
#define F_CONTROL_SCALE         128L
#define F_I_CONTROL_SCALE       4096L
#define F_KP_Q                  160L
#define F_KI_Q                  12L
#define F_KD_Q                  384L
#define F_I_TERM_LIMIT          5L
#define F_INTEGRAL_LIMIT        ((F_I_TERM_LIMIT * ADC_SCALE * F_I_CONTROL_SCALE) / F_KI_Q)

#define MOVE_KP_Q               128L
#define MOVE_BOOST_MAX          12L
#define MOVE_DEADBAND           2L
#define PREDICT_TICKS           4L

#define HOLD_ENTER_ERROR_Q      (2L * ADC_SCALE)
#define HOLD_EXIT_ERROR_Q       (4L * ADC_SCALE)
#define HOLD_ENTER_SPEED_Q      6
#define HOLD_EXIT_SPEED_Q       12
#define HOLD_CONFIRM_TICKS      3U
#define HOLD_MIN_AGE_TICKS      6U
#define I_ENABLE_ERROR_Q        (3L * ADC_SCALE)
#define I_DEADBAND_Q            2

#define LEARN_DEADBAND          3
#define LEARN_DIVISOR           4
#define LEARN_STEP_MAX          3
#define LEARN_BIAS_LIMIT        160
#define LEARN_WINDOW            4U
#define LEARN_MIN_AGE           8U
#define LEARN_SPEED_MAX_Q       8
#define LEARN_SPAN_MAX_Q        (2L * ADC_SCALE)

/* new: motor off if GPIO17 has not changed for 200 ms (0 disables) */
#define FLIP_SYNC_TIMEOUT_TICKS 400U

#define F_STATE_BACK            0U
#define F_STATE_FRONT           1U
#define F_STATE_NONE            2U

/* ================================================================
 * Shared state
 * ================================================================ */
static volatile uint8_t mode = MODE_PID;
static volatile uint16_t control_tick = 0U;
static volatile uint16_t raw_adc = 0U;
static volatile uint16_t pwm_actual = 0U;
static volatile uint32_t uptime_ms = 0UL;
static volatile uint16_t sample_miss = 0U;
static volatile uint16_t adc_busy_count = 0U;   /* kept for log format */
static volatile uint16_t control_max_ticks = 0U;
static volatile uint16_t rx_errors = 0U;
static volatile uint8_t sensor_fault = 0U;
static volatile uint8_t fault_report = 0U;
static volatile uint8_t print_request = 0U;

static volatile uint8_t sync_state = 0U;        /* INT4 edge copy (S cmd) */
static volatile uint16_t sync_edge_count = 0U;

/* ================================================================
 * CAL state (names as in VCM_Control_Updated.c)
 * ================================================================ */
static volatile uint8_t target_valid = 0U;
static volatile int16_t cmd_target = 0;
static volatile int16_t reach_min = DEFAULT_REACH_MIN;
static volatile int16_t reach_max = DEFAULT_REACH_MAX;
static volatile uint8_t range_valid = 0U;
static volatile uint16_t kp_q = KP_DEFAULT;
static volatile uint16_t ki_q = KI_DEFAULT;
static volatile uint16_t kd_q = KD_DEFAULT;
static volatile uint16_t gain_q8 = GAIN_DEFAULT;
static volatile uint8_t dither_amp = DITHER_DEFAULT;
static volatile uint16_t current_adc = 0U;
static volatile uint16_t pwm_center = 0U;
static volatile int16_t base_pwm = PWM_START;
static volatile int16_t control_error = 0;
static volatile uint8_t stable_flag = 0U;
static volatile uint8_t stable_report = 0U;
static volatile uint8_t unreach_flag = 0U;
static volatile uint8_t unreach_report = 0U;
static volatile uint16_t heartbeat_count = 0U;
static volatile uint16_t clamp_count = 0U;
static volatile int32_t integral_q8 = 0L;
static volatile int32_t position_q8 = 0L;
static volatile int32_t velocity_q8 = 0L;
static volatile int32_t reference_q8 = 0L;
static volatile int32_t center_q8 = 0L;
static volatile int32_t hold_center_q8 = 0L;
static volatile uint16_t stable_count = 0U;
static volatile uint16_t sat_count = 0U;
static volatile uint8_t filter_ready = 0U;
static uint16_t raw_prev1 = 0U, raw_prev2 = 0U;
static uint16_t median_prev = 0U;
static volatile uint16_t rail_count = 0U;
static uint8_t dither_phase = 0U;

static volatile uint8_t range_phase = 0U;
static uint16_t range_elapsed = 0U, range_samples = 0U;
static uint32_t range_sum = 0UL;
static int16_t measured_low = 0, measured_high = 0;
static volatile uint8_t range_report = 0U;
static volatile uint8_t range_ok = 0U;
static volatile uint8_t deferred_valid = 0U;
static volatile int16_t deferred_target = 0;

/* ================================================================
 * FLIP state (Fast_Hold names with f_ prefix)
 * ================================================================ */
static volatile uint16_t f_front_target = 0U;
static volatile uint16_t f_back_target = 0U;
static volatile uint8_t f_targets_valid = 0U;
static volatile uint16_t f_heartbeat_count = 0U;

static volatile uint16_t f_current_adc = 0U;
static volatile uint16_t f_target_adc = 0U;
static volatile uint16_t f_pwm_cmd = 0U;
static volatile int16_t f_control_error = 0;
static volatile int16_t f_p_term = 0;
static volatile int16_t f_i_term = 0;
static volatile int16_t f_d_term = 0;
static volatile uint8_t f_state = F_STATE_NONE;

static uint8_t f_phase_arrived = 0U;
static uint8_t f_hold_confirm = 0U;
static uint16_t f_phase_age = 0U;

static int16_t f_filtered_q = 0;
static int16_t f_last_q = 0;
static int16_t f_velocity_q = 0;
static uint8_t f_derivative_ready = 0U;
static uint8_t f_filter_ready = 0U;

static int16_t f_tail_error_q[LEARN_WINDOW];
static uint16_t f_tail_speed_q[LEARN_WINDOW];
static uint8_t f_tail_saturated[LEARN_WINDOW];
static uint8_t f_tail_index = 0U;
static uint8_t f_tail_count = 0U;

static int32_t f_front_integral = 0L;
static int32_t f_back_integral = 0L;
static int16_t f_front_bias = 0;
static int16_t f_back_bias = 0;

static uint8_t f_last_sync = 0U;
static volatile uint16_t f_sync_idle = FLIP_SYNC_TIMEOUT_TICKS;

/* UART1 RX ring */
static volatile uint8_t rx_ring[RX_SIZE];
static volatile uint8_t rx_head = 0U, rx_tail = 0U;
static volatile uint8_t rx_bad_line = 0U;

/* ================================================================
 * Helpers
 * ================================================================ */
static int32_t clamp32(int32_t x, int32_t lo, int32_t hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static int32_t abs32(int32_t x) { return x < 0L ? -x : x; }

static int16_t abs16(int16_t x) { return x < 0 ? (int16_t)(-x) : x; }

static inline __attribute__((always_inline))
int32_t round_divide(int32_t value, int32_t divisor)
{
    if (value >= 0) return (value + divisor / 2L) / divisor;
    return -((-value + divisor / 2L) / divisor);
}

static void count_event(volatile uint16_t *p)
{
    if (*p != 65535U) ++*p;
}

static uint16_t median3(uint16_t a, uint16_t b, uint16_t c)
{
    uint16_t t;
    if (a > b) { t = a; a = b; b = t; }
    if (b > c) { t = b; b = c; c = t; }
    if (a > b) b = a;
    return b;
}

/* ================================================================
 * Hardware layer (shared)
 * ================================================================ */
static void st_write(uint8_t on)
{
    if (on) PORTD |= (1U << ST_PIN);
    else PORTD &= (uint8_t)~(1U << ST_PIN);
}

/* PWM=0 is truly LOW: Fast PWM OCR=0 alone otherwise emits narrow pulses. */
static void set_pwm_raw(int16_t pwm)
{
    pwm = (int16_t)clamp32(pwm, PWM_MIN, PWM_MAX);
    pwm_actual = (uint16_t)pwm;
    if (pwm == 0) {
        TCCR1A &= (uint8_t)~((1U << COM1A1) | (1U << COM1A0));
        PORTB &= (uint8_t)~(1U << PWM_PIN);
        OCR1A = 0U;
    } else {
        OCR1A = (uint16_t)pwm;
        TCCR1A = (uint8_t)((TCCR1A & ~(1U << COM1A0)) | (1U << COM1A1));
    }
}

static void motor_enable(void)
{
    PORTB &= (uint8_t)~(1U << DIR_PIN);
    PORTB |= (1U << SLEEP_PIN);
}

static void motor_disable(void)
{
    set_pwm_raw(0);
    PORTB &= (uint8_t)~(1U << SLEEP_PIN);
}

static uint16_t adc_read(void)
{
    ADCSRA |= (1U << ADSC);
    while (ADCSRA & (1U << ADSC)) { }
    return ADC;
}

/* ================================================================
 * CAL controller (VCM_Control_Updated.c logic, runs at 1 kHz)
 * ================================================================ */
static void write_center(int32_t value_q8)
{
    int16_t c, amp;
    center_q8 = clamp32(value_q8, 0L, PWM_MAX * Q8);
    c = (int16_t)((center_q8 + Q8 / 2L) / Q8);
    pwm_center = (uint16_t)c;
    amp = dither_amp;
    if (amp > c) amp = c;
    if (amp > PWM_MAX - c) amp = (int16_t)(PWM_MAX - c);
    dither_phase ^= 1U;
    set_pwm_raw((int16_t)(c + (dither_phase ? amp : -amp)));
}

static void reset_settling(void)
{
    stable_count = 0U;
    stable_flag = 0U;
    stable_report = 0U;
    sat_count = 0U;
    unreach_flag = 0U;
    unreach_report = 0U;
    st_write(0U);
}

static void cal_filter_update(uint16_t raw)
{
    uint16_t m;
    int32_t pair_q8, old_position, instant_velocity;
    if (!filter_ready) {
        raw_prev1 = raw_prev2 = median_prev = raw;
        position_q8 = (int32_t)raw * Q8;
        velocity_q8 = 0L;
        filter_ready = 1U;
    } else {
        m = median3(raw, raw_prev1, raw_prev2);
        raw_prev2 = raw_prev1;
        raw_prev1 = raw;
        /* Two phases average the 500 Hz dither. */
        pair_q8 = ((int32_t)m + median_prev) * (Q8 / 2L);
        median_prev = m;
        old_position = position_q8;
        position_q8 += (pair_q8 - position_q8) / 2L;
        instant_velocity = position_q8 - old_position;
        velocity_q8 += (instant_velocity - velocity_q8) / 4L;
    }
    current_adc = (uint16_t)((position_q8 + Q8 / 2L) / Q8);
}

static int32_t limit_center(int32_t request, int32_t previous, int32_t velocity)
{
    int32_t delta, slew = PWM_SLEW_Q8;
    request = clamp32(request, 0L, PWM_MAX * Q8);
    delta = request - previous;
    /* Negative plant gain: +PWM brakes positive ADC velocity. */
    if ((velocity >= BRAKE_SPEED_Q8 && delta > 0L) ||
        (velocity <= -BRAKE_SPEED_Q8 && delta < 0L))
        slew = PWM_BRAKE_SLEW_Q8;
    return clamp32(previous + clamp32(delta, -slew, slew), 0L, PWM_MAX * Q8);
}

static void update_stable_state(int32_t final_error)
{
    uint8_t in_band = (uint8_t)(reference_q8 == (int32_t)cmd_target * Q8 &&
        abs32(final_error) <= POSITION_TOL_Q8);
    if (in_band) {
        if (stable_count < STABLE_MS) ++stable_count;
        if (stable_count >= STABLE_MS && !stable_flag) {
            stable_flag = 1U;
            stable_report = 1U;
            hold_center_q8 = center_q8;
        }
    } else {
        stable_count = 0U;
        if (reference_q8 != (int32_t)cmd_target * Q8 ||
            abs32(final_error) > STABLE_EXIT_TOL_Q8) {
            stable_flag = 0U;
            stable_report = 0U;
        }
    }
    if (stable_flag && in_band)
        hold_center_q8 += (center_q8 - hold_center_q8) / 16L;
    st_write(stable_flag);
}

static void position_control_update(void)
{
    int32_t e, final_e, p, d, di, new_i, request, output;
    int32_t old_center = center_q8;
    int32_t goal = (int32_t)cmd_target * Q8;

    reference_q8 += clamp32(goal - reference_q8, -REF_STEP_Q8, REF_STEP_Q8);
    e = reference_q8 - position_q8;
    final_e = goal - position_q8;
    p = ((int32_t)kp_q * e) / CONTROL_SCALE;
    d = ((int32_t)kd_q * velocity_q8) / CONTROL_SCALE;

    if (ki_q == 0U) integral_q8 = 0L;
    di = 0L;
    if (ki_q != 0U && abs32(final_e) > POSITION_TOL_Q8) {
        uint8_t approaching_fast = (uint8_t)(
            (final_e > 0L && velocity_q8 > BRAKE_SPEED_Q8) ||
            (final_e < 0L && velocity_q8 < -BRAKE_SPEED_Q8));
        if (!approaching_fast)
            di = ((int32_t)ki_q * clamp32(e, -I_ERROR_LIMIT_Q8,
                                         I_ERROR_LIMIT_Q8)) / I_CONTROL_SCALE;
    }
    new_i = clamp32(integral_q8 + di, -I_LIMIT_Q8, I_LIMIT_Q8);
    request = (int32_t)base_pwm * Q8 - p - new_i + d;
    output = limit_center(request, old_center, velocity_q8);

    if ((di > 0L && request < output) || (di < 0L && request > output)) {
        new_i = integral_q8;
        request = (int32_t)base_pwm * Q8 - p - new_i + d;
        output = limit_center(request, old_center, velocity_q8);
    }
    integral_q8 = new_i;
    write_center(output);
    control_error = (int16_t)(cmd_target - (int16_t)current_adc);
    update_stable_state(final_e);

    if (abs32(final_e) > STABLE_EXIT_TOL_Q8 &&
        abs32(velocity_q8) <= STABLE_SPEED_Q8 &&
        ((final_e < 0L && center_q8 >= (PWM_MAX - 1L) * Q8) ||
         (final_e > 0L && center_q8 <= Q8))) {
        if (sat_count < SAT_TIMEOUT_MS) ++sat_count;
        if (sat_count >= SAT_TIMEOUT_MS && !unreach_flag) {
            unreach_flag = 1U;
            unreach_report = 1U;
        }
    } else {
        sat_count = 0U;
        unreach_flag = 0U;
        unreach_report = 0U;
    }
}

static uint8_t set_reach_limits(int16_t lo, int16_t hi)
{
    int16_t margin = EDGE_MARGIN, lower, upper;
    if ((int16_t)(hi - lo) < REACH_MIN_SPAN) return 0U;
    while (margin > 0 && hi - lo - 2 * margin < REACH_MIN_SPAN) --margin;
    lower = (int16_t)(lo + margin);
    upper = (int16_t)(hi - margin);
    if (lower < 100) lower = 100;
    if (upper > 1023) upper = 1023;
    if (upper - lower < REACH_MIN_SPAN) return 0U;
    reach_min = lower;
    reach_max = upper;
    range_valid = 1U;
    return 1U;
}

static void range_control_update(void)
{
    ++range_elapsed;
    if (range_phase == 1U || range_phase == 2U) {
        write_center(range_phase == 1U ? 0L : PWM_MAX * Q8);
        if (range_elapsed > RANGE_SETTLE_MS) {
            range_sum += current_adc;
            ++range_samples;
            if (range_samples >= RANGE_SAMPLES) {
                int16_t average = (int16_t)(range_sum / RANGE_SAMPLES);
                if (range_phase == 1U) measured_high = average;
                else measured_low = average;
                ++range_phase;
                range_elapsed = range_samples = 0U;
                range_sum = 0UL;
                write_center(range_phase == 2U ? PWM_MAX * Q8 : 0L);
            }
        }
    } else {
        write_center(0L);
        if (range_elapsed >= RANGE_RETURN_MS) {
            range_ok = set_reach_limits(measured_low, measured_high);
            integral_q8 = 0L;
            base_pwm = PWM_START;
            reference_q8 = position_q8;
            reset_settling();
            target_valid = 0U;
            range_phase = 0U;
            mode = MODE_PID;
            range_report = 1U;
        }
    }
}

/* ================================================================
 * FLIP controller (Adaptive_Base_V3_Fast_Hold.c logic, runs at 2 kHz)
 * ================================================================ */
static void f_reset_phase_tracking(void)
{
    f_phase_arrived = 0U;
    f_hold_confirm = 0U;
    f_phase_age = 0U;
    f_tail_index = 0U;
    f_tail_count = 0U;
}

static void f_filter_update(uint16_t raw)
{
    int16_t raw_q = (int16_t)((int32_t)raw * ADC_SCALE);
    int16_t delta_q;
    if (!f_filter_ready) {
        f_filtered_q = raw_q;
        f_filter_ready = 1U;
    } else {
        f_filtered_q += (int16_t)round_divide((int32_t)raw_q - f_filtered_q, 2L);
    }
    if (!f_derivative_ready) {
        f_velocity_q = 0;
        f_derivative_ready = 1U;
    } else {
        delta_q = f_filtered_q - f_last_q;
        f_velocity_q = (int16_t)round_divide((int32_t)f_velocity_q + delta_q, 2L);
    }
    f_last_q = f_filtered_q;
    f_current_adc = (uint16_t)round_divide(f_filtered_q, ADC_SCALE);
}

static int16_t f_learning_delta(int16_t end_error)
{
    int16_t abs_error = abs16(end_error);
    int16_t magnitude;
    if (abs_error <= LEARN_DEADBAND) return 0;
    magnitude = (int16_t)((abs_error + LEARN_DIVISOR - 1) / LEARN_DIVISOR);
    if (magnitude < 1) magnitude = 1;
    if (magnitude > LEARN_STEP_MAX) magnitude = LEARN_STEP_MAX;
    return (end_error > 0) ? (int16_t)(-magnitude) : magnitude;
}

static void f_record_learning_sample(int16_t error_q, uint8_t saturated)
{
    f_tail_error_q[f_tail_index] = error_q;
    f_tail_speed_q[f_tail_index] = (uint16_t)abs16(f_velocity_q);
    f_tail_saturated[f_tail_index] = saturated;
    f_tail_index++;
    if (f_tail_index == LEARN_WINDOW) f_tail_index = 0U;
    if (f_tail_count < LEARN_WINDOW) f_tail_count++;
}

static void f_update_cycle_learning(uint8_t finished_state)
{
    int32_t error_sum = 0;
    uint32_t speed_sum = 0UL;
    int16_t minimum, maximum, mean_error, learn_delta;
    int16_t *learn_bias;
    int32_t *integral_acc;
    int32_t new_bias;
    uint8_t i;

    if (f_tail_count < LEARN_WINDOW || f_phase_age < LEARN_MIN_AGE) return;
    minimum = maximum = f_tail_error_q[0];
    for (i = 0U; i < LEARN_WINDOW; i++) {
        if (f_tail_saturated[i]) return;
        error_sum += f_tail_error_q[i];
        speed_sum += f_tail_speed_q[i];
        if (f_tail_error_q[i] < minimum) minimum = f_tail_error_q[i];
        if (f_tail_error_q[i] > maximum) maximum = f_tail_error_q[i];
    }
    if (speed_sum > (uint32_t)LEARN_SPEED_MAX_Q * LEARN_WINDOW ||
        (int32_t)maximum - minimum > LEARN_SPAN_MAX_Q)
        return;
    if (finished_state == F_STATE_FRONT) {
        learn_bias = &f_front_bias;
        integral_acc = &f_front_integral;
    } else if (finished_state == F_STATE_BACK) {
        learn_bias = &f_back_bias;
        integral_acc = &f_back_integral;
    } else return;

    mean_error = (int16_t)round_divide(error_sum, LEARN_WINDOW * ADC_SCALE);
    learn_delta = f_learning_delta(mean_error);
    if (learn_delta != 0) {
        new_bias = clamp32((int32_t)*learn_bias + learn_delta,
                           -LEARN_BIAS_LIMIT, LEARN_BIAS_LIMIT);
        if (new_bias != *learn_bias) {
            *learn_bias = (int16_t)new_bias;
            *integral_acc = 0;
        }
    }
}

static uint16_t f_adaptive_base(uint8_t state)
{
    uint16_t target = (state == F_STATE_FRONT) ? f_front_target : f_back_target;
    int16_t bias = (state == F_STATE_FRONT) ? f_front_bias : f_back_bias;
    int32_t base = (int32_t)BASE_REF_PWM +
        ((int32_t)BASE_REF_ADC - (int32_t)target) * (int32_t)BASE_PWM_PER_ADC;
    base = clamp32(base, PWM_MIN, F_PWM_MAX);
    base = clamp32(base + bias, PWM_MIN, F_PWM_MAX);
    return (uint16_t)base;
}

static uint16_t f_calculate_pwm(uint16_t target, uint16_t base, uint8_t state)
{
    int16_t error_q = (int16_t)((int32_t)target * ADC_SCALE - f_filtered_q);
    int16_t abs_error_q = abs16(error_q);
    int16_t abs_speed_q = abs16(f_velocity_q);
    int32_t predicted_error_q;
    int32_t boost_q = 0L;
    int32_t p_q, i_q, d_q, command_q;
    int32_t *integral_acc;
    int32_t old_integral, candidate_integral;
    uint8_t saturated;

    if (f_phase_age != 65535U) f_phase_age++;

    /* Hold needs small error AND low speed; reversible. */
    if (f_phase_arrived &&
        (abs_error_q >= HOLD_EXIT_ERROR_Q || abs_speed_q >= HOLD_EXIT_SPEED_Q)) {
        f_phase_arrived = 0U;
        f_hold_confirm = 0U;
    }
    if (!f_phase_arrived) {
        if (f_phase_age >= HOLD_MIN_AGE_TICKS &&
            abs_error_q <= HOLD_ENTER_ERROR_Q && abs_speed_q <= HOLD_ENTER_SPEED_Q) {
            if (f_hold_confirm < HOLD_CONFIRM_TICKS) f_hold_confirm++;
            if (f_hold_confirm >= HOLD_CONFIRM_TICKS) f_phase_arrived = 1U;
        } else f_hold_confirm = 0U;
    }

    integral_acc = (state == F_STATE_FRONT) ? &f_front_integral : &f_back_integral;
    old_integral = *integral_acc;
    candidate_integral = old_integral;
    if (f_phase_arrived && abs_error_q > I_DEADBAND_Q &&
        abs_error_q <= I_ENABLE_ERROR_Q && abs_speed_q <= HOLD_ENTER_SPEED_Q)
        candidate_integral += error_q;
    candidate_integral = clamp32(candidate_integral, -F_INTEGRAL_LIMIT, F_INTEGRAL_LIMIT);

    /* Bounded approach boost from 2 ms predicted remaining distance. */
    predicted_error_q = (int32_t)error_q - PREDICT_TICKS * f_velocity_q;
    if (abs_error_q > MOVE_DEADBAND * ADC_SCALE) {
        if (error_q > 0 && predicted_error_q > MOVE_DEADBAND * ADC_SCALE)
            boost_q = (predicted_error_q - MOVE_DEADBAND * ADC_SCALE) * MOVE_KP_Q / F_CONTROL_SCALE;
        else if (error_q < 0 && predicted_error_q < -MOVE_DEADBAND * ADC_SCALE)
            boost_q = (predicted_error_q + MOVE_DEADBAND * ADC_SCALE) * MOVE_KP_Q / F_CONTROL_SCALE;
    }
    boost_q = clamp32(boost_q, -MOVE_BOOST_MAX * ADC_SCALE, MOVE_BOOST_MAX * ADC_SCALE);

    p_q = F_KP_Q * error_q / F_CONTROL_SCALE;
    i_q = F_KI_Q * candidate_integral / F_I_CONTROL_SCALE;
    d_q = F_KD_Q * f_velocity_q / F_CONTROL_SCALE;
    command_q = (int32_t)base * ADC_SCALE - p_q - boost_q - i_q + d_q;

    if ((command_q < PWM_MIN * ADC_SCALE && error_q > 0) ||
        (command_q > (int32_t)F_PWM_MAX * ADC_SCALE && error_q < 0)) {
        candidate_integral = old_integral;
        i_q = F_KI_Q * candidate_integral / F_I_CONTROL_SCALE;
        command_q = (int32_t)base * ADC_SCALE - p_q - boost_q - i_q + d_q;
    }
    *integral_acc = candidate_integral;

    saturated = (uint8_t)(command_q <= PWM_MIN * ADC_SCALE ||
                          command_q >= (int32_t)F_PWM_MAX * ADC_SCALE);
    f_record_learning_sample(error_q, saturated);

    f_control_error = (int16_t)target - (int16_t)f_current_adc;
    f_p_term = (int16_t)round_divide(p_q + boost_q, ADC_SCALE);
    f_i_term = (int16_t)round_divide(i_q, ADC_SCALE);
    f_d_term = (int16_t)round_divide(d_q, ADC_SCALE);

    command_q = clamp32(command_q, PWM_MIN * ADC_SCALE, (int32_t)F_PWM_MAX * ADC_SCALE);
    return (uint16_t)round_divide(command_q, ADC_SCALE);
}

static void f_output_off(void)
{
    f_state = F_STATE_NONE;
    f_target_adc = 0U;
    f_pwm_cmd = 0U;
    f_control_error = 0;
    f_p_term = f_i_term = f_d_term = 0;
    f_reset_phase_tracking();
    motor_disable();
}

static void flip_control_update(uint8_t sync_sample)
{
    uint8_t new_state;
    uint16_t target, base;

    if (sync_sample != f_last_sync) {
        f_last_sync = sync_sample;
        f_sync_idle = 0U;
    } else if (f_sync_idle != 65535U) {
        f_sync_idle++;
    }

    if (!f_targets_valid) {
        f_output_off();
        return;
    }

#if FLIP_SYNC_TIMEOUT_TICKS > 0
    if (f_sync_idle >= FLIP_SYNC_TIMEOUT_TICKS) {
        /* Mango stopped (no X): no edges -> motor off, targets kept. */
        f_output_off();
        return;
    }
#endif

    new_state = sync_sample ? F_STATE_BACK : F_STATE_FRONT;
    target = sync_sample ? f_back_target : f_front_target;

    if (new_state != f_state) {
        if (f_state == F_STATE_FRONT || f_state == F_STATE_BACK)
            f_update_cycle_learning(f_state);
        f_state = new_state;
        f_reset_phase_tracking();
    }

    base = f_adaptive_base(new_state);
    f_target_adc = target;
    f_pwm_cmd = f_calculate_pwm(target, base, new_state);
    motor_enable();
    set_pwm_raw((int16_t)f_pwm_cmd);
}

/* Called with interrupts disabled. Returns 1 when targets changed. */
static uint8_t flip_apply_targets(uint16_t front, uint16_t back)
{
    if (f_targets_valid && mode == MODE_FLIP &&
        f_front_target == front && f_back_target == back) {
        count_event(&f_heartbeat_count);
        return 0U;
    }
    f_front_target = front;
    f_back_target = back;
    f_front_integral = f_back_integral = 0L;
    f_front_bias = f_back_bias = 0;
    f_reset_phase_tracking();
    f_state = F_STATE_NONE;
    f_sync_idle = FLIP_SYNC_TIMEOUT_TICKS;   /* wait for the first edge */
    f_targets_valid = 1U;
    return 1U;
}

static void flip_invalidate(void)
{
    f_targets_valid = 0U;
    f_state = F_STATE_NONE;
    f_reset_phase_tracking();
}

/* ================================================================
 * Mode transitions (called with interrupts disabled)
 * ================================================================ */
static void leave_cal_for_flip(void)
{
    reset_settling();          /* ST low */
    target_valid = 0U;
    deferred_valid = 0U;
    range_report = 0U;
    range_phase = 0U;
    integral_q8 = 0L;
}

static void leave_flip_for_cal(void)
{
    flip_invalidate();
    /* CAL apply_target starts its slew from the current mean output. */
    pwm_center = pwm_actual;
    center_q8 = (int32_t)pwm_actual * Q8;
    target_valid = 0U;
    mode = MODE_PID;
}

static void apply_target(int16_t t)
{
    int16_t original = t;
    if (t < reach_min) t = reach_min;
    if (t > reach_max) t = reach_max;
    if (t != original) count_event(&clamp_count);
    if (target_valid && mode == MODE_PID && t == cmd_target) {
        count_event(&heartbeat_count);
        return;
    }
    if (!target_valid || mode == MODE_STOP) {
        reference_q8 = position_q8;
        integral_q8 = 0L;
        base_pwm = PWM_START;
        center_q8 = (int32_t)pwm_center * Q8;
    } else if (stable_flag && stable_count >= STABLE_MS) {
        int32_t ff = 0L;
        if (gain_q8 >= GAIN_MIN)
            ff = clamp32(((position_q8 - (int32_t)t * Q8) * Q8) /
                         (int32_t)gain_q8, -FF_MAX_PWM * Q8, FF_MAX_PWM * Q8);
        base_pwm = (int16_t)clamp32((hold_center_q8 + ff + Q8 / 2L) / Q8,
                                   PWM_MIN, PWM_MAX);
        integral_q8 = 0L;
        reference_q8 = position_q8;
    }
    cmd_target = t;
    target_valid = 1U;
    reset_settling();
    mode = MODE_PID;
    motor_enable();
}

static void start_range(void)
{
    if (mode == MODE_FLIP) flip_invalidate();
    reset_settling();
    target_valid = 0U;
    deferred_valid = 0U;
    range_report = 0U;
    range_ok = 0U;
    integral_q8 = 0L;
    center_q8 = 0L;
    pwm_center = 0U;
    range_phase = 1U;
    range_elapsed = range_samples = 0U;
    range_sum = 0UL;
    mode = MODE_RANGE;
    motor_enable();
    set_pwm_raw(0);
}

static void stop_motor(void)
{
    mode = MODE_STOP;
    flip_invalidate();
    deferred_valid = 0U;
    range_report = 0U;
    range_phase = 0U;
    reset_settling();
    integral_q8 = 0L;
    center_q8 = 0L;
    pwm_center = 0U;
    /* target_valid kept: R resumes the last CAL target, as in the original */
    motor_disable();
}

static void soft_reset(void)
{
    if (rail_count != 0U) return;
    sensor_fault = 0U;
    fault_report = 0U;
    if (mode == MODE_RANGE) {
        stop_motor();
        target_valid = 0U;
    }
    reset_settling();
    reference_q8 = position_q8;
    base_pwm = (int16_t)pwm_center;
    integral_q8 = clamp32(((int32_t)kd_q * velocity_q8) / CONTROL_SCALE,
                          -I_LIMIT_Q8, I_LIMIT_Q8);
    if (ki_q == 0U) integral_q8 = 0L;
    mode = MODE_PID;
    if (target_valid) motor_enable();
    else set_pwm_raw(0);
}

/* ================================================================
 * 1 kHz CAL step (every 2nd tick): filter + fault always, control in CAL
 * ================================================================ */
static void cal_step(uint16_t raw)
{
    cal_filter_update(raw);

    if (raw <= 2U || raw >= 1021U) {
        if (rail_count < SENSOR_RAIL_MS) ++rail_count;
    } else rail_count = 0U;

    if (rail_count >= SENSOR_RAIL_MS && !sensor_fault) {
        sensor_fault = 1U;
        fault_report = 1U;
        range_report = 0U;
        deferred_valid = 0U;
        range_phase = 0U;
        flip_invalidate();
        mode = MODE_STOP;
        reset_settling();
        motor_disable();
    }

    if (mode == MODE_FLIP) return;          /* FLIP drives the output */

    control_error = (int16_t)(cmd_target - (int16_t)current_adc);
    if (sensor_fault || mode == MODE_STOP) {
        pwm_center = 0U;
        center_q8 = 0L;
        motor_disable();
        st_write(0U);
    } else if (mode == MODE_RANGE) {
        range_control_update();
        st_write(0U);
    } else if (!target_valid) {
        center_q8 = 0L;
        pwm_center = 0U;
        set_pwm_raw(0);
        st_write(0U);
    } else position_control_update();
}

/* ================================================================
 * Interrupts
 * ================================================================ */
ISR(USART1_RX_vect)
{
    static uint8_t discard_to_eol = 0U;
    uint8_t status = UCSR1A;
    uint8_t c = UDR1;
    uint8_t next = (uint8_t)((rx_head + 1U) & RX_MASK);
    if ((status & ((1U << FE1) | (1U << DOR1) | (1U << UPE1))) || next == rx_tail) {
        count_event(&rx_errors);
        rx_head = rx_tail;
        rx_bad_line = 1U;
        discard_to_eol = 1U;
        return;
    }
    if (discard_to_eol) {
        if (c != '\r' && c != '\n') return;
        discard_to_eol = 0U;
    }
    rx_ring[rx_head] = c;
    rx_head = next;
}

/* Edge counter for the S command only. Control samples PE4 itself. */
ISR(INT4_vect)
{
    sync_state = (PINE & (1U << SYNC_PIN)) ? 1U : 0U;
    count_event(&sync_edge_count);
}

ISR(TIMER0_COMP_vect)
{
    static uint16_t previous_clock = 0U;
    static uint16_t remainder = 0U;
    uint16_t start = TCNT3;
    uint16_t elapsed = (uint16_t)(start - previous_clock);
    uint32_t accumulated;
    uint16_t raw;
    uint8_t sync_sample;
    uint16_t exec;

    previous_clock = start;
    if (elapsed > MISS_THRESHOLD_COUNTS) count_event(&sample_miss);
    accumulated = (uint32_t)remainder + elapsed;
    while (accumulated >= 2000UL) {       /* Timer3 /8: 2000 counts = 1 ms */
        accumulated -= 2000UL;
        ++uptime_ms;
    }
    remainder = (uint16_t)accumulated;

    raw = adc_read();
    /* INT4 is blocked during this ISR; sample the level here. */
    sync_sample = (PINE & (1U << SYNC_PIN)) ? 1U : 0U;
    raw_adc = raw;
    ++control_tick;

    f_filter_update(raw);                 /* 2 kHz, every mode */
    if (control_tick & 1U) cal_step(raw); /* 1 kHz, every mode */

    if (mode == MODE_FLIP) {
        if (!sensor_fault) flip_control_update(sync_sample);
    } else {
        /* keep the edge tracker current so FLIP entry sees real idle time */
        if (sync_sample != f_last_sync) {
            f_last_sync = sync_sample;
            f_sync_idle = 0U;
        } else if (f_sync_idle != 65535U) {
            f_sync_idle++;
        }
    }

    print_request = 1U;

    exec = (uint16_t)(TCNT3 - start);
    if (exec > control_max_ticks) control_max_ticks = exec;
}

/* ================================================================
 * UART output
 * ================================================================ */
static void uart_char(uint8_t port, char c)
{
    if (port == 0U) {
        while (!(UCSR0A & (1U << UDRE0))) { }
        UDR0 = (uint8_t)c;
    } else {
        while (!(UCSR1A & (1U << UDRE1))) { }
        UDR1 = (uint8_t)c;
    }
}

static void uart_flash(uint8_t port, const char *s)
{
    char c;
    while ((c = (char)pgm_read_byte(s++)) != '\0') uart_char(port, c);
}
#define SAY(port, literal) uart_flash((port), PSTR(literal))

static void uart_unsigned(uint8_t port, uint32_t n)
{
    char digits[10];
    uint8_t count = 0U;
    do {
        digits[count++] = (char)('0' + n % 10UL);
        n /= 10UL;
    } while (n != 0UL);
    while (count != 0U) uart_char(port, digits[--count]);
}

static void uart_signed(uint8_t port, int32_t n)
{
    if (n < 0L) {
        uart_char(port, '-');
        uart_unsigned(port, (uint32_t)(-(n + 1L)) + 1UL);
    } else uart_unsigned(port, (uint32_t)n);
}

/* ================================================================
 * Init
 * ================================================================ */
static void uart_init(void)
{
    DDRE |= (1U << PE1);
    DDRD = (uint8_t)((DDRD & ~(1U << PD2)) | (1U << PD3) | (1U << ST_PIN));
    UCSR0A = (1U << U2X0);
    UBRR0H = (uint8_t)(UBRR_VALUE >> 8);
    UBRR0L = (uint8_t)UBRR_VALUE;
    UCSR0B = (1U << TXEN0);
    UCSR0C = (1U << UCSZ01) | (1U << UCSZ00);
    UCSR1A = (1U << U2X1);
    UBRR1H = (uint8_t)(UBRR_VALUE >> 8);
    UBRR1L = (uint8_t)UBRR_VALUE;
    UCSR1B = (1U << RXEN1) | (1U << RXCIE1) | (1U << TXEN1);
    UCSR1C = (1U << UCSZ11) | (1U << UCSZ10);
}

static void hardware_init(void)
{
    DDRB |= (1U << PWM_PIN) | (1U << DIR_PIN) | (1U << SLEEP_PIN);
    PORTB &= (uint8_t)~((1U << PWM_PIN) | (1U << DIR_PIN) | (1U << SLEEP_PIN));
    TCCR1A = (1U << WGM11);
    TCCR1B = (1U << WGM13) | (1U << WGM12) | (1U << CS10);
    ICR1 = PWM_TOP;
    OCR1A = 0U;
    uart_init();
    st_write(0U);

    DDRF &= (uint8_t)~(1U << PF1);
    PORTF &= (uint8_t)~(1U << PF1);
    ADMUX = (1U << REFS0) | HALL_CH;
    /* 125 kHz ADC clock, polled in the timer ISR (no ADC interrupt). */
    ADCSRA = (1U << ADEN) | (1U << ADPS2) | (1U << ADPS1) | (1U << ADPS0);

    /* PE4 input, no pull-up; INT4 on any change (edge counter). */
    DDRE &= (uint8_t)~(1U << SYNC_PIN);
    PORTE &= (uint8_t)~(1U << SYNC_PIN);
    sync_state = (PINE & (1U << SYNC_PIN)) ? 1U : 0U;
    f_last_sync = sync_state;
    EICRB = (uint8_t)((EICRB & ~(1U << ISC41)) | (1U << ISC40));
    EIFR = (1U << INTF4);
    EIMSK |= (1U << INT4);
}

static void timers_start(void)
{
    TCCR3A = 0U;
    TCCR3B = (1U << CS31);                /* free-running /8, 0.5 us */
    TCNT3 = 0U;
    ASSR &= (uint8_t)~(1U << AS0);
    TCCR0 = 0U;
    TCNT0 = 0U;
    OCR0 = TIMER0_OCR;
    TIFR = (1U << OCF0);
    TIMSK |= (1U << OCIE0);
    TCCR0 = (1U << WGM01) | (1U << CS01) | (1U << CS00);   /* ATmega128: 011 = /32 */
}

/* ================================================================
 * Command parser
 * ================================================================ */
static uint8_t parse_value(const char *s, uint16_t *out)
{
    uint32_t value = 0UL;
    uint8_t digits = 0U;
    while (*s == ' ' || *s == '\t' || *s == ':' || *s == '=') ++s;
    while (*s >= '0' && *s <= '9') {
        value = value * 10UL + (uint8_t)(*s++ - '0');
        if (++digits > 5U || value > 65535UL) return 0U;
    }
    while (*s == ' ' || *s == '\t') ++s;
    if (!digits || *s != '\0') return 0U;
    *out = (uint16_t)value;
    return 1U;
}

/* T<n> or T<a>,<b> / T<a>:<b>. Returns 1 = single, 2 = pair, 0 = error. */
static uint8_t parse_target(const char *s, uint16_t *a, uint16_t *b)
{
    uint32_t value = 0UL;
    uint8_t digits = 0U;
    while (*s == ' ' || *s == '\t' || *s == ':' || *s == '=') ++s;
    while (*s >= '0' && *s <= '9') {
        value = value * 10UL + (uint8_t)(*s++ - '0');
        if (++digits > 5U || value > 65535UL) return 0U;
    }
    if (!digits) return 0U;
    *a = (uint16_t)value;
    while (*s == ' ' || *s == '\t') ++s;
    if (*s == '\0') return 1U;
    if (*s != ',' && *s != ':') return 0U;
    ++s;
    return parse_value(s, b) ? 2U : 0U;
}

static void send_position(void)
{
    uint8_t saved = SREG, st;
    uint16_t adc;
    cli(); adc = current_adc; st = stable_flag; SREG = saved;
    SAY(1U, "POS:"); uart_unsigned(1U, adc); uart_char(1U, ':');
    uart_unsigned(1U, st); SAY(1U, "\r\n");
}

static void send_targets(void)
{
    uint8_t saved = SREG, valid;
    uint16_t front, back;
    cli();
    front = f_front_target; back = f_back_target; valid = f_targets_valid;
    SREG = saved;
    SAY(1U, "TARGETS:"); uart_unsigned(1U, front); uart_char(1U, ':');
    uart_unsigned(1U, back); uart_char(1U, ':');
    uart_unsigned(1U, valid); SAY(1U, "\r\n");
}

static void send_sync(void)
{
    uint8_t saved = SREG, level;
    uint16_t edges;
    cli(); level = sync_state; edges = sync_edge_count; SREG = saved;
    SAY(1U, "SYNC:"); uart_unsigned(1U, level); uart_char(1U, ':');
    uart_unsigned(1U, edges); SAY(1U, "\r\n");
}

/* CAL single target, shared by T<n> and T<a>,<a>. Interrupts disabled. */
static uint8_t command_single_target(uint16_t value)
{
    if (value < 100U || value > 1023U || sensor_fault || !filter_ready) return 0U;
    if (mode == MODE_FLIP) leave_flip_for_cal();
    if (mode == MODE_RANGE || range_report) {
        deferred_target = (int16_t)value;
        deferred_valid = 1U;
    } else apply_target((int16_t)value);
    return 1U;
}

static void process_command(const char *line)
{
    char command;
    uint8_t saved, error = 0U, kind, changed = 0U;
    uint16_t value = 0U, second = 0U;

    while (*line == ' ' || *line == '\t') ++line;
    if (*line == '\0') return;
    command = *line++;
    if (command >= 'a' && command <= 'z') command = (char)(command - 'a' + 'A');

    if (command == 'T') {
        kind = parse_target(line, &value, &second);
        if (kind == 0U) { SAY(1U, "ERR:CMD\r\n"); return; }
        saved = SREG;
        cli();
        if (kind == 1U || value == second) {
            /* division20 T<n>, or Mango contrast hold T<a>,<a> */
            if (!command_single_target(value)) error = 1U;
        } else if (value < TARGET_ADC_MIN || value > TARGET_ADC_MAX ||
                   second < TARGET_ADC_MIN || second > TARGET_ADC_MAX) {
            error = 2U;
        } else if (sensor_fault || mode == MODE_RANGE || range_report) {
            error = 1U;                    /* not during M or a fault */
        } else {
            /* Order-free: GPIO17 HIGH always shows the HIGH-ADC image,
               so the larger value is the GPIO17-HIGH (BACK slot) target. */
            uint16_t low_adc = (value < second) ? value : second;
            uint16_t high_adc = (value < second) ? second : value;
            if (mode != MODE_FLIP) leave_cal_for_flip();
            changed = flip_apply_targets(low_adc, high_adc);
            mode = MODE_FLIP;
        }
        SREG = saved;
        if (error == 2U) SAY(1U, "ERR:RANGE\r\n");
        else if (error) SAY(1U, "ERR:CMD\r\n");
        else if (changed) send_targets();
        return;
    }

    if (command == 'P' || command == 'I' || command == 'D' ||
        command == 'G' || command == 'A') {
        if (!parse_value(line, &value)) { SAY(1U, "ERR:CMD\r\n"); return; }
    } else {
        while (*line == ' ' || *line == '\t') ++line;
        if (*line != '\0') { SAY(1U, "ERR:CMD\r\n"); return; }
    }

    if (command == 'Q') {
        if (mode == MODE_FLIP) send_targets();
        else send_position();
        return;
    }
    if (command == 'S') { send_sync(); return; }

    saved = SREG;
    cli();
    switch (command) {
    case 'M':
        if (sensor_fault) error = 1U;
        else if (mode != MODE_RANGE) start_range();
        break;
    case 'X': stop_motor(); break;
    case 'R':
        if (rail_count != 0U || mode == MODE_FLIP) error = 1U;
        else soft_reset();
        break;
    case 'G':
        if (value != 0U && (value < GAIN_MIN || value > GAIN_MAX)) error = 1U;
        else gain_q8 = value;
        break;
    case 'A':
        if (value > DITHER_MAX) error = 1U;
        else dither_amp = (uint8_t)value;
        break;
    case 'P':
        if (value > KP_MAX) error = 1U;
        else kp_q = value;
        break;
    case 'I':
        if (value > KI_MAX) error = 1U;
        else { ki_q = value; if (value == 0U) integral_q8 = 0L; }
        break;
    case 'D':
        if (value > KD_MAX) error = 1U;
        else kd_q = value;
        break;
    default: error = 1U; break;
    }
    SREG = saved;
    if (error) SAY(1U, "ERR:CMD\r\n");
    else if (command == 'X') SAY(1U, "STOPPED\r\n");
}

static void service_rx(void)
{
    static char line[CMD_SIZE];
    static uint8_t length = 0U, discard = 0U;
    while (rx_tail != rx_head) {
        uint8_t saved = SREG;
        char c;
        cli();
        if (rx_bad_line) { discard = 1U; length = 0U; rx_bad_line = 0U; }
        c = (char)rx_ring[rx_tail];
        rx_tail = (uint8_t)((rx_tail + 1U) & RX_MASK);
        SREG = saved;
        if (c == '\r' || c == '\n') {
            if (!discard && length) {
                line[length] = '\0';
                process_command(line);
            }
            length = 0U;
            discard = 0U;
        } else if (!discard) {
            if (length < CMD_SIZE - 1U) line[length++] = c;
            else { length = 0U; discard = 1U; }
        }
    }
}

static void service_reports(void)
{
    uint8_t saved, rr, ok, sr, ur, fr;
    uint16_t adc;
    int16_t lo, hi;
    saved = SREG;
    cli();
    rr = range_report; ok = range_ok;
    sr = (uint8_t)(stable_report && stable_flag);
    ur = (uint8_t)(unreach_report && unreach_flag);
    fr = fault_report;
    stable_report = unreach_report = fault_report = 0U;
    adc = current_adc; lo = reach_min; hi = reach_max;
    SREG = saved;
    if (fr) SAY(1U, "FAULT:ADC\r\n");
    if (rr) {
        if (!ok) SAY(1U, "ERR:RANGE\r\n");
        SAY(1U, "LIMIT:"); uart_signed(1U, lo); uart_char(1U, ':');
        uart_signed(1U, hi); SAY(1U, "\r\nRANGE:");
        uart_signed(1U, lo); uart_char(1U, ':');
        uart_signed(1U, hi); SAY(1U, "\r\n");
        saved = SREG;
        cli();
        range_report = 0U;
        if (deferred_valid && !sensor_fault) {
            int16_t t = deferred_target;
            deferred_valid = 0U;
            apply_target(t);
        }
        SREG = saved;
    }
    if (ur) {
        SAY(1U, "UNREACH:"); uart_unsigned(1U, adc); SAY(1U, "\r\n");
    }
    if (sr) {
        SAY(1U, "STABLE:"); uart_unsigned(1U, adc); SAY(1U, "\r\n");
    }
}

/* CAL/RANGE/STOP log: identical fields to VCM_Control_Updated.c */
static void print_status_cal(void)
{
    uint8_t saved = SREG, st, md, fault;
    uint16_t adc, raw, pwm, center, g, hb, cl, miss, busy, rx, exec;
    int16_t target, base, error;
    int32_t i, v;
    uint32_t ms;
    cli();
    target = cmd_target; adc = current_adc; raw = raw_adc;
    pwm = pwm_actual; center = pwm_center; base = base_pwm;
    error = control_error; i = integral_q8; g = gain_q8;
    hb = heartbeat_count; cl = clamp_count; st = stable_flag;
    ms = uptime_ms; v = velocity_q8; md = mode; fault = sensor_fault;
    miss = sample_miss; busy = adc_busy_count; rx = rx_errors;
    exec = control_max_ticks;
    SREG = saved;
    SAY(0U, "Target:"); uart_signed(0U, target);
    SAY(0U, " | ADC:"); uart_unsigned(0U, adc);
    SAY(0U, " | Error:"); uart_signed(0U, error);
    SAY(0U, " | PWM:"); uart_unsigned(0U, pwm);
    SAY(0U, " | Base:"); uart_signed(0U, base);
    SAY(0U, " | I:"); uart_signed(0U, i / Q8);
    SAY(0U, " | G:"); uart_unsigned(0U, g);
    SAY(0U, " | K:0 | BA:0 | CL:"); uart_unsigned(0U, cl);
    SAY(0U, " | HB:"); uart_unsigned(0U, hb);
    SAY(0U, " | ST:"); uart_unsigned(0U, st);
    SAY(0U, " | t:"); uart_unsigned(0U, ms);
    SAY(0U, " | Raw:"); uart_unsigned(0U, raw);
    SAY(0U, " | Center:"); uart_unsigned(0U, center);
    SAY(0U, " | V:"); uart_signed(0U, v * 1000L / Q8);
    SAY(0U, " | Mode:"); uart_unsigned(0U, md);
    SAY(0U, " | Fault:"); uart_unsigned(0U, fault);
    SAY(0U, " | Miss:"); uart_unsigned(0U, miss);
    SAY(0U, " | AdcBusy:"); uart_unsigned(0U, busy);
    SAY(0U, " | RxErr:"); uart_unsigned(0U, rx);
    SAY(0U, " | ExecUs:"); uart_unsigned(0U, exec / 2U);
    SAY(0U, "\r\n");
}

/* FLIP log: identical to Adaptive_Base_V3_Fast_Hold.c
   STATE,ADC,ERROR,P,I,D,PWM,TICK   (N = no drive: no targets / no sync) */
static void print_status_flip(void)
{
    uint8_t saved = SREG, state;
    uint16_t adc, pwm, tick;
    int16_t error, p, i, d;
    cli();
    state = f_state; adc = f_current_adc; error = f_control_error;
    p = f_p_term; i = f_i_term; d = f_d_term; pwm = f_pwm_cmd;
    tick = control_tick;
    print_request = 0U;
    SREG = saved;
    if (state == F_STATE_FRONT) SAY(0U, "F,");
    else if (state == F_STATE_BACK) SAY(0U, "B,");
    else SAY(0U, "N,");
    uart_unsigned(0U, adc); uart_char(0U, ',');
    uart_signed(0U, error); uart_char(0U, ',');
    uart_signed(0U, p); uart_char(0U, ',');
    uart_signed(0U, i); uart_char(0U, ',');
    uart_signed(0U, d); uart_char(0U, ',');
    uart_unsigned(0U, pwm); uart_char(0U, ',');
    uart_unsigned(0U, tick);
    SAY(0U, "\r\n");
}

int main(void)
{
    uint32_t last_print = 0UL;
    cli();
    hardware_init();
    motor_disable();
    _delay_ms(20);
    timers_start();
    sei();
    SAY(0U, "\r\nVCM unified: CAL 1ms PI-D dither70 | FLIP 0.5ms Fast_Hold\r\n");
    SAY(0U, "T<n>=cal  T<a>,<b>=flip (order-free: GPIO17 HIGH=larger ADC)  M X Q S R P/I/D/G/A\r\n");
    SAY(1U, "READY:VCM_UNIFIED_V2\r\n");
    while (1) {
        uint8_t saved;
        uint32_t now;
        service_rx();
        service_reports();
        if (mode == MODE_FLIP) {
            if (print_request) print_status_flip();
        } else {
            saved = SREG;
            cli(); now = uptime_ms; SREG = saved;
            if ((uint32_t)(now - last_print) >= PRINT_PERIOD_MS) {
                last_print = now;
                print_status_cal();
            }
        }
    }
}
