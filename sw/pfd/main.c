#include <stdio.h>
#include <stdarg.h>
#include <stdbool.h>

#include "ch32fun.h"

#include "i2c.h"

/*
 * GPSDO PFD controller software implementation
 *
 * 1. Counts the number of OCXO cycles, thus
 *    tracking the phase of the OCXO output.
 *
 * 2. Receives PPS pulses from the GNSS receiver unit.
 *
 * 3. Receives absolute time information related to each PPS
 *    as data from the GNSS interface.
 *
 * 4. Compares PPS pulse timing to OCXO timing to control
 *    OCXO for accurate time-keeping.
 *
 * 5. Produces a low jitter PPS output.
 *
 * Implements two controllers: FLL and PLL
 * FLL is used initially to quickly tune the OCXO close
 * to the correct frequency, without caring about phase.
 * PLL is then used to keep the OCXO also phase locked to
 * the TAI second.
 *
 * The GNSS PPS is very jittery in the short term.
 * Thus the phase and/or frequency errors are heavily filtered.
 * The used FLL and PLL controllers are purposely designed to work
 * with such filtered error signals.
 *
 * The OCXO runs at 10 MHz, but it is clock doubled to 20 MHz.
 * At each 20 MHz cycle, a tick counter (OCXO phase) is incremented.
 * The low 16 bits of the tick counter are handled via hardware,
 * while during overflow, a 64 bit software counter is incremented.
 *
 * At each PPS pulse rising edge, input capture hardware is used
 * to capture the 16 bit tick counter value at the start of the pulse.
 * The full 64 bit tick count  (OCXO phase) is then computed by adding
 * the 64 bit overflow counter.
 *
 * The PPS pulses need to be additionally validated against
 * erroneous pulses arriving from the GNSS receiver unit
 *
 * Two controllers are implemented: a frequency-locked loop (FLL)
 * controller and a phase-locked loop (PLL) controller.
 *
 * Both controllers average the observed tick error over a number of PPS
 * pulses. From the averaged tick error, a phase error estimate is derived.
 * See https://kiedontaa.blogspot.com/2025/02/on-quantization-errors-and-recovering.html
 * A frequency error estimate is also derived based on the change of the phase error.
 *
 * The FLL controller is used first to tune the OCXO frequency.
 * The FLL starts out with a short averaging period to achieve a quick
 * coarse adjustment of the frequency, and then gradually increases
 * the averaging period to improve the resolution of the frequency error.
 *
 * Once the FLL has converged to a good frequency, a target phase value is sampled
 * and the control is switched to a PLL controller. The PLL controller is based on
 * https://kiedontaa.blogspot.com/2024/07/single-parameter-controller-for-gps.html
 */

// tick counter high bits [63:16], bits 15:0 always 0.
volatile uint64_t tick_high = 0;

// tick value of last PPS pulse
volatile uint64_t last_tick_pps;
// was the last PPS pulse valid
volatile bool last_valid_pps = false;

/**
 * Get the current tick counter value.
 * Try to account for the fact that the full (sw + hw) tick count
 * cannot be read atomically.
 *
 * I'm not absolutely sure that this routine is water tight.
 * If the TIM1 overflow happens at just the right time,
 * it might be possible to produce an incorrect tick count
 */
uint64_t get_current_tick(void) {
    __disable_irq();
    uint16_t hw_tick = TIM1->CNT;
    uint64_t sw_tick = tick_high;
    uint16_t tim1_intfr = TIM1->INTFR;
    uint16_t hw_tick_late = TIM1->CNT;
    __enable_irq();

    // check if the TIM1 overflow was set
    if (tim1_intfr & TIM_UIF) {
        /*
         * The overflow could have occurred at three critical times:
         *  1. After interrupts disabled, but before hw_tick read
         *  2. After hw_tick read, but before/during sw_tick read
         *  3. After sw_tick read
         *
         * For cases 2 and 3, we have hw_tick_late < hw_tick.
         * In those cases hw_tick is consistent with sw_tick,
         * as hw_tick represents the pre-overflow value and
         * the ISR for updating the sw_tick hasn't executed yet.
         *
         * In case 1, however, hw_tick represents the post-overflow
         * value, but sw_tick represents a pre-overflow value.
         */

        if (hw_tick < hw_tick_late) {
            sw_tick += 65536;
        }
    }

    return sw_tick + hw_tick;
}

/*
 * I2C slave receive (push to us)
 */

/*
 * The variables below represent the latest fully received values.
 * These values get updated once an entire I2C transfer to us is complete.
 */
volatile uint32_t last_timestamp = 0; // last received timestamp value
volatile bool last_timestamp_valid = false; // was the last received timestamp valid

// data structure for pushing data to us
volatile struct {
    uint32_t timestamp; // unix time stamp
    uint8_t valid; // 1 = time stamp is valid, otherwise time stamp is invalid
} __attribute__((packed)) rx_data_buf;

// tick value of last received timestamp information
volatile uint64_t last_timestamp_tick = 0;

// called when someone has pushed data to us
void rx_done_callback(void) {
    last_timestamp_tick = get_current_tick();

    // double buffer the data so that it changes atomically
    last_timestamp = rx_data_buf.timestamp;
    last_timestamp_valid = rx_data_buf.valid == 1;
}

/*
 * I2C slave transmit (pulled from us)
 */

/*
 * The variables below represent the latest values to be transmitted.
 * These values are used to construct the I2C transfer data when requested.
 */
volatile int32_t ui_phase_error_raw; // raw phase error after a cycle, 3.0 fs res.
volatile int32_t ui_phase_error_filtered; // filtered phase error, 3.0 fs res.
volatile int32_t ui_frequency_error_raw; // raw frequency error after a cycle, 29.8 nHz res.
volatile int32_t ui_frequency_error_filtered; // filtered phase error, 29.8 nHz res.
volatile uint64_t ui_tick_valid; // tick of latest OCXO control update
volatile int16_t ui_ocxo_control_word; // last OCXO control word
volatile uint8_t ui_control_mode; // controller mode at last update, 0=FLL, 1=PLL

// data structure for pulling data from us
volatile struct {
    int32_t phase_error_raw; // Q7.24, LSB ~= 3.0 fs
    int32_t phase_error_filtered; // Q7.24, LSB ~= 3.0 fs
    int32_t frequecy_error_raw; // Q7.24, LSB ~= 29.8 nHz
    int32_t frequency_error_filtered; // Q7.24, LSB ~= 29.8 nHz
    uint32_t time_since_valid; // How many seconds since last control update (i.e. since valid GPS data)
    int16_t ocxo_control_word; // OCXO control word
    uint8_t control_mode; // Which control mode are we in 0=FLL, 1=PLL
} __attribute__((packed)) tx_data_buf;

// called when someone wants to pull data from us
void tx_start_callback(void) {
    // double buffer the data so that it changes atomically
    tx_data_buf.phase_error_raw = ui_phase_error_raw;
    tx_data_buf.phase_error_filtered = ui_phase_error_filtered;
    tx_data_buf.frequecy_error_raw = ui_frequency_error_raw;
    tx_data_buf.frequency_error_filtered = ui_frequency_error_filtered;
    tx_data_buf.time_since_valid = (get_current_tick() - ui_tick_valid) / 20000000UL;
    tx_data_buf.ocxo_control_word = ui_ocxo_control_word;
    tx_data_buf.control_mode = ui_control_mode;
}

// tick count for producing the next PPS output
volatile uint64_t next_pps_out_tick;

/**
 * Timer 1 update interrupt
 *
 * Handles 64 bit tick counter high bits incrementing.
 *
 * Additionally handles a part of the PPS output generation.
 */
void TIM1_UP_IRQHandler(void) __attribute__((interrupt));
void TIM1_UP_IRQHandler(void) {
    // increment high bits of tick counter by one full cycle of TIM1
    tick_high += 65536;

    // if a PPS output time passed by already, increment it by a second
    if (next_pps_out_tick < tick_high) {
        next_pps_out_tick += 20000000ULL;
    }

    /*
     * The following checks whether a PPS output is set to occur within counts
     * 32768 through 65535 of this TIM1 period.
     *
     * If this is the case, it configures CC2 to go high at compare match in
     * order to produce the PPS out pulse.
     */
    if (next_pps_out_tick >= tick_high + 32768 && next_pps_out_tick < tick_high + 65536) {
        // a PPS out will occur during the latter half of this TIM1 wraparound period

        // update compare channel to produce
        TIM1->CH2CVR = next_pps_out_tick - tick_high;
        TIM1->CHCTLR1 = TIM_OC2M_0; // set output when match
    }
    TIM1->INTFR = ~TIM_UIF;
}

// minimum tick interval between two PPS pulses which is considered valid
const int minimum_pps_valid_tick_interval = 19999900;
// maximum tick interval between two PPS pulses which is considered valid
const int maximum_pps_valid_tick_interval = 20000100;

/**
 * Timer 1 capture-compare interrupt
 *
 * Handles PPS input capture timing.
 * Attempts to handle the issues caused by non-atomicity of the full
 * (sw + hw) tick counting.
 *
 * Additionally handles a part of the PPS output generation.
 */
void TIM1_CC_IRQHandler(void) __attribute__((interrupt));
void TIM1_CC_IRQHandler(void) {
    if (TIM1->INTFR & TIM_CC4IF) {
        uint16_t hw_tick = TIM1->CNT;
        uint16_t tim1_intfr = TIM1->INTFR;
        uint64_t sw_tick = tick_high;
        uint16_t capture_value = TIM1->CH4CVR;

        /*
         * Critical moments for TIM1 overflow to occur:
         *  1. Before CC4 capture.
         *  2. After CC4 capture, but before entering this ISR
         *  3. After entering this ISR, but before reading hw_tick
         *  4. After reading hw_tick, but before reading tim1_intfr
         *
         * In case 1, the ISR for updating sw_tick would have
         * executed before entering this ISR. Thus the captured value
         * and sw_tick are consistent and nothing extra needs to be done.
         *
         * In case 2, the ISR for updating sw_tick would have
         * executed before entering this ISR. However, the captured value
         * is from before the overflow. Thus the value must be adjusted.
         *
         * In cases 3-4, the ISR for updating sw_tick would not have
         * executed before entering this ISR. Thus the captured value
         * and sw_tick are consistent and nothing extra needs to be done.
         *
         * Case 2 and 3 are characterized by hw_tick being less than the
         * the captured value. However, in case 2 the ISR has been serviced
         * and thus the update interrupt flag is cleared, while in case 3
         * it is set.
         *
         * In case 4, hw_tick is greater than the captured value, but the
         * update interrupt flag is set.
         */

        uint64_t new_tick = sw_tick + capture_value;

        if (hw_tick < capture_value) {
            // case 2 or case 3 has occurred
            if (!(tim1_intfr & TIM_UIF)) {
                // case 2 occurred
                // compensate for the extra count in sw_tick
                new_tick -= 65536;
            }
        }

        // compute the number of ticks since the last input PPS
        const int last_difftick_pps = new_tick - last_tick_pps;
        // update latest input PPS tick time
        last_tick_pps = new_tick;

        // perform validation of pulse interval
        if (last_difftick_pps >= minimum_pps_valid_tick_interval &&
            last_difftick_pps <= maximum_pps_valid_tick_interval) {
            last_valid_pps = true;
        } else {
            last_valid_pps = false;
        }

        TIM1->INTFR = ~TIM_CC4IF;
    }

    /*
     * If CC2 was configured to go high on a compare match on this cycle, i.e.
     * to produce a PPS out pulse, then the following will configure CC2 to
     * go back low 65536 counts (= 3.2768 ms) later.
     *
     * If CC2 was not configured to go high on this cycle, this just keeps
     * CC2 low.
     */
    if (TIM1->INTFR & TIM_CC2IF) {
        TIM1->CHCTLR1 = TIM_OC2M_1; // clear output when match
        TIM1->INTFR = ~TIM_CC2IF;
    }

    /*
     * The following is triggered by count reaching 32768.
     * It checks whether a PPS output is set to occur within counts 0
     * through 32767 of the next TIM1 period.
     *
     * If this is the case, it configures CC2 to go high at compare match in
     * order to produce the PPS out pulse.
     */
    if (TIM1->INTFR & TIM_CC1IF) {
        if (next_pps_out_tick >= tick_high + 65536 && next_pps_out_tick < tick_high + 98304) {
            // a PPS out will occur during the first half of the next wraparound period

            TIM1->CH2CVR = next_pps_out_tick - (tick_high + 65536);
            TIM1->CHCTLR1 = TIM_OC2M_0; // set output when match
        }
        TIM1->INTFR = ~TIM_CC1IF;
    }
}

/**
 * Simple floor function (does not support large values)
 * Implement here to omit need for standard math library.
 * @param value Value to round down to nearest integer
 * @returns Floored value
 */
float floorf(float value) {
    if (value == (float)((int)value)) {
        return value; // already an integer
    }

    if (value > 0.0f) {
        return (float)((int)value); // positive value, just truncate
    }

    return (float)((int)value - 1); // negative value, truncate and subtract one
}
/**
 * Estimate phase error from averaged tick error.
 * Phase is given in number of ticks, but corrected for the sampling bias and
 * with phase 0 occurring at tick_error_average = 0.5.
 * See https://kiedontaa.blogspot.com/2025/02/on-quantization-errors-and-recovering.html
 * @param tick_error Average of tick error
 * @returns Estimated phase error
 */
float estimate_phase_error(const float tick_error) {
    /*
     * Interpolant data for bias correction.
     * Table below is derived for 4.5 nanoseconds RMS phase jitter using sw/support/phase_error.py script.
     */
    const size_t N = 32;
    const float avg_fract[32] = {
        0.00000000e+00, 8.73564864e-08, 6.39587402e-07, 3.71330501e-06,
        1.87771334e-05, 8.37818527e-05, 3.30784732e-04, 1.15723290e-03,
        3.59220477e-03, 9.90969424e-03, 2.43432771e-02, 5.33827548e-02,
        1.04833732e-01, 1.85111059e-01, 2.95414730e-01, 4.28885827e-01,
        5.71114173e-01, 7.04585270e-01, 8.14888941e-01, 8.95166268e-01,
        9.46617245e-01, 9.75656723e-01, 9.90090306e-01, 9.96407795e-01,
        9.98842767e-01, 9.99669215e-01, 9.99916218e-01, 9.99981223e-01,
        9.99996287e-01, 9.99999360e-01, 9.99999913e-01, 1.00000000e+00
    };
    const float phase_fract[32] = {
        -0.50000000, -0.46774194, -0.43548387, -0.40322581,
        -0.37096774, -0.33870968, -0.30645161, -0.27419355,
        -0.24193548, -0.20967742, -0.17741935, -0.14516129,
        -0.11290323, -0.08064516, -0.04838710, -0.01612903,
         0.01612903,  0.04838710,  0.08064516,  0.11290323,
         0.14516129,  0.17741935,  0.20967742,  0.24193548,
         0.27419355,  0.30645161,  0.33870968,  0.37096774,
         0.40322581,  0.43548387,  0.46774194,  0.50000000
    };

    const float tick_error_floor = floorf(tick_error);
    const float tick_error_fract = tick_error - tick_error_floor;

    // lookup the phase error in the table with linear interpolation

    // value is below the first entry, return the first entry
    if (tick_error_fract <= avg_fract[0]) {
        return tick_error_floor + phase_fract[0];
    }

    // try to find the right entry in the table
    for (size_t i = 1; i < N; i++) {
        if (tick_error_fract <= avg_fract[i]) {
            const float c = (tick_error_fract - avg_fract[i - 1]) / (avg_fract[i] - avg_fract[i - 1]);
            return tick_error_floor + (1.0f - c) * phase_fract[i - 1] + c * phase_fract[i];
        }
    }

    // value is above the last entry, return the last entry
    return tick_error_floor + phase_fract[N-1];
}

/**
 * Kahan-Babushka summation.
 * Given value a, where a = a_coarse + a_fine, compute a += b.
 * @param a_coarse Pointer to coarse part of value a. Value is updated according to sum.
 * @param a_fine Pointer to fine part of value a. Value is updated according to sum.
 * @param b Value to add to a.
 */
void kahan_babushka_sum(float* a_coarse, float* a_fine, const float b) {
    // See https://en.wikipedia.org/wiki/Kahan_summation_algorithm
    const float fine_diff = b - (*a_fine);
    const float sum_coarse = (*a_coarse) + fine_diff;

    *a_fine = (sum_coarse - (*a_coarse)) - fine_diff;
    *a_coarse = sum_coarse;
}

int main() {
    SystemInit();

    RCC->APB2PCENR |= RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC | RCC_APB2Periph_TIM1;
    RCC->APB1PCENR |= RCC_APB1Periph_I2C1;

    // PA2: Cleaned PPS output
    GPIOA->CFGLR &= ~(0xf << (4 * 2));
    GPIOA->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_PP_AF) << (4 * 2);

    // PC4: GNSS PPS input capture
    GPIOC->CFGLR &= ~(0xf << (4 * 4));
    GPIOC->CFGLR |= (GPIO_Speed_In | GPIO_CNF_IN_FLOATING) << (4 * 4);

    // Initialize I2C as address 0x10 with 100 kHz speed
    i2c_init(0x10, 100000UL);
    i2c_set_slave_rx(&rx_data_buf, sizeof(rx_data_buf), rx_done_callback);
    i2c_set_slave_tx(&tx_data_buf, sizeof(tx_data_buf), tx_start_callback);

    /*
     * Configure TIM1. TIM1 runs the show. TIM1 counts ticks at 20 MHz.
     * CH4 is used as input capture, capturing the tick count at PPS input.
     * CH2 is used as output compare, producing the PPS output.
     * Overflow interrupt increments tick counter high bits and updates CH2 for PPS output.
     * CH1 is configured to produce an interupt at mid period to update CH2 for PPS output.
     */
    TIM1->PSC = 0; // no prescaling => count at 20 MHz
    TIM1->CHCTLR1 = 0; // No input/output on CH1, while CH2 is configured later dynamically.
    TIM1->CHCTLR2 = TIM_CC4S_0; // No input/output on CH3, CH4 captures from TIM1_CH4 pin without filtering
    TIM1->CCER = TIM_CC4E | TIM_CC2NE; // Enable rising edge input on CH4, enable output on CH2N
    TIM1->CH1CVR = 32768; // CH1 compare interrupt at mid period
    TIM1->DMAINTENR = TIM_UIE | TIM_CC4IE | TIM_CC2IE | TIM_CC1IE; // Enable interrupts
    TIM1->BDTR = TIM_MOE; // Master output enable
    TIM1->CTLR1 = TIM_CEN; // Enable counter

    NVIC_EnableIRQ(TIM1_UP_IRQn); // Enable overflow interrupt
    NVIC_EnableIRQ(TIM1_CC_IRQn); // Enable CC interrupts

    /* General state variables */
    uint8_t state = 0; // 0=FLL, 1=PLL, 2=Free running
    int64_t tick_offset = 0; // tick offset to adjust initial tick error to 0
    bool tick_offset_initialized = false; // has the tick offset been initialized

    /* Phase and frequency sample averaging parameters */
    const size_t tick_error_average_count_fll_start = 4; // how many tick_error samples to initially average in FLL
    const size_t tick_error_average_count_fll_end = 256; // how many tick_error samples to at most average in FLL before switching to PLL
    const size_t tick_error_average_count_pll = 8; // how many tick_error samples to average in PLL

    // number of tick error samples to average
    size_t tick_error_average_count = tick_error_average_count_fll_start;

    int tick_error_sum = 0; // accumulated sum of tick error sample values
    float phase_error_start = 0; // phase error at start of averaging period
    size_t tick_error_count = 0; // accumulated number of tick error samples

    /**
     * OCXO control gain.
     * The OCXO response is of the form:
     * frequency_ocxo = control_word / ocxo_gain + frequency_offset
     */
    const float ocxo_gain = 4363.0f;

    /* Filtered frequency and phase errors */
    float alpha = 1 / 32.0f; // low-pass filter damping factor for filtered errors
    float freq_error_filtered = 0;
    float phase_error_filtered = 0;

    /**
     * Frequency and phase error integrals
     * The integrals grow slowly, but can end up being large in value.
     * Single precision floating points do not have enough accuracy for long time-scales.
     * The integrals are kept as value = coarse + fine, and accumulated using Kahan-Babushka summation.
     */
    float freq_error_integral_coarse = 0;
    float freq_error_integral_fine = 0;
    float phase_error_integral_coarse = 0;
    float phase_error_integral_fine = 0;

    /* FLL controller parameters */
    const float error_damping_factor_fll = 0.125f;
    const float i_fll = ocxo_gain * error_damping_factor_fll;

    /* PLL controller parameters */
    const float r = alpha / 3.0f; // system matrix eigenvalue
    const float p_pll = 8.0f * ocxo_gain * r / (9.0f * tick_error_average_count_pll);
    const float i_pll = ocxo_gain * r * r / (3.0f * tick_error_average_count_pll);


    // make sure OCXO is at known state
    ui_ocxo_control_word = 0;
    i2c_master_transfer(0x0c, &ui_ocxo_control_word, sizeof(ui_ocxo_control_word));

    while(1) {
        /*
         * Poll for valid PPS and timestamp data.
         */

        bool valid_pps; // was the last PPS input pulse valid
        uint64_t tick_pps; // when did we receive the last PPS input pulse

        bool valid_timestamp; // was the last GNSS timestamp valid
        uint64_t tick_timestamp; // when did we receive the last GNSS timestamp
        uint32_t timestamp; // what was the last GNSS timestamp value

        __disable_irq(); // read the following data atomically
        valid_pps = last_valid_pps;

        if (valid_pps) {
            tick_pps = last_tick_pps;
            tick_timestamp = last_timestamp_tick;
            valid_timestamp = last_timestamp_valid;
            timestamp = last_timestamp;

            // invalidate the values for the next cycle
            last_valid_pps = false;
            last_timestamp_valid = false;
        }
        __enable_irq();

        if (!valid_pps || !valid_timestamp) {
            // no valid PPS or timestamp data, wait for next cycle
            continue;
        }



        /*
         * Validate received PPS and timestamp data compatiblity.
         *
         * The GNSS timestamp transmision takes a while, so the last timestamp
         * received is nominally the absolute time of the previously received PPS pulse.
         *
         * Thus, after receiving the PPS, we check that the last timestamp arrived
         * within a reasonable time in the past, i.e. not too far in the past,
         * but not too recently either. To be exact, the time stamp must have occured:
         *  - Later than 1 second in the past
         *  - Earlier than 0.5 seconds in the past
         */

        const int64_t pps_timestamp_tick_diff = tick_pps - tick_timestamp; // positive values are toward the past

        if (pps_timestamp_tick_diff > 20000000L || pps_timestamp_tick_diff < 10000000L) {
            // the received timestamp is too far in the past or too recent, wait for next cycle
            continue;
        }



        /*
         * Compute tick error between GNSS and OCXO and accumulate it into an average
         */

        // compute the absolute tick value of the PPS input pulse
        const int64_t tick_pps_absolute = timestamp * 20000000LL;

        if (!tick_offset_initialized) {
            tick_offset_initialized = true;
            // initialize tick_offset so that tick_error starts at 0
            tick_offset = tick_pps_absolute - tick_pps;
        }

        // compute the error between the last PPS input absolute tick and when it was observed
        const int tick_error = tick_pps_absolute - tick_pps - tick_offset;

        // Accumulate an average of the tick errors over an averaging period
        tick_error_sum += tick_error;
        tick_error_count++;

        // update latest tick validity, to report good status in UI
        // note: this operation is atomic, so no need to acquire i2c
        ui_tick_valid = tick_pps;

#if (FUNCONF_USE_DEBUGPRINTF)
        const int freq_err_scale = freq_error_filtered * 1000.0f;
        const int phase_err_scale = phase_error_filtered * 1000.0f;

        printf("%d, %lu, %d, %d, %d, %d, %d\n", state, timestamp, (int)tick_error_count, (int)freq_err_scale, (int)tick_error, (int)phase_err_scale, (int)ui_ocxo_control_word);
#endif


        if (tick_error_count < tick_error_average_count) {
            // not enough samples yet, wait for next cycle
            continue;
        }



        /*
         * Compute phase error estimate from averaged tick error
         */

        // Compute the average tick error over the averaging period
        const float tick_error_average = (float)tick_error_sum / (float)tick_error_average_count;
        // Estimate the phase error from the average tick error
        const float phase_error = estimate_phase_error(tick_error_average);
        // Estimate frequency error from the drift of the phase
        const float freq_error = (phase_error - phase_error_start) / (float)tick_error_average_count;

        freq_error_filtered += alpha * (freq_error - freq_error_filtered);
        phase_error_filtered += alpha * (phase_error - phase_error_filtered);

        float control = 0.0f;

        if (state == 0) {
            // Compute freq_error_integral += freq_err
            kahan_babushka_sum(&freq_error_integral_coarse, &freq_error_integral_fine, freq_error);

            control = i_fll * freq_error_integral_coarse;

            /*
             * Check if we want to adjust the averaging period
             */
            if (phase_error - phase_error_start > -0.25f && phase_error - phase_error_start < 0.25f) {
                // increase averaging time for next cycle to improve resolution
                tick_error_average_count *= 2;
            }

            if (phase_error - phase_error_start < -1.0f || phase_error - phase_error_start > 1.0f) {
                if (tick_error_average_count > tick_error_average_count_fll_start) {
                    // decrease averaging time for next cycle to improve speed
                    tick_error_average_count /= 2;
                }
            }


            /*
             * Check if it's time to switch to PLL
             */
            if (tick_error_average_count > tick_error_average_count_fll_end) {
                // switch to PLL
                state = 1;

                // switch to PLL averaging
                tick_error_average_count = tick_error_average_count_pll;

                // reset tick error to zero
                tick_offset = tick_pps_absolute - tick_pps;

                // reset phase error to correspond with constant zero tick error
                phase_error_filtered = -0.5f;

                // seamless hand over to PLL
                phase_error_integral_coarse = (control - p_pll * phase_error_filtered) / i_pll;
                phase_error_integral_fine = 0.0f;
            }
        } else if (state == 1) {
            // Compute phase_error_integral += phase_error
            kahan_babushka_sum(&phase_error_integral_coarse, &phase_error_integral_fine, phase_error);

            control = p_pll * phase_error_filtered + i_pll * phase_error_integral_coarse;
        } else if (state == 2) {
            // free running, no control
            control = ui_ocxo_control_word;
        }

        if (control > 32767.0f) {
            control = 32767.0f;
        }
        if (control < -32768.0f) {
            control = -32768.0f;
        }

        // get ready to accumulate next period
        tick_error_sum = 0;
        tick_error_count = 0;
        phase_error_start = phase_error;

        const int16_t ocxo_control_word = control;

        /*
         * Update monitoring data for UI
         */
        i2c_acquire(); // don't allow i2c accesses while we update the following data

        ui_frequency_error_raw = freq_error * 16777216.0f;
        ui_phase_error_raw = phase_error * 16777216.0f;
        ui_frequency_error_filtered = freq_error_filtered * 16777216.0f;
        ui_phase_error_filtered = phase_error_filtered * 16777216.0f;

        ui_ocxo_control_word = ocxo_control_word;

        ui_control_mode = state;
        i2c_release();

        /*
         * Update absolute tick value of the next PPS output pulse.
         */
        __disable_irq();
        next_pps_out_tick = tick_pps_absolute - tick_offset + 20000000;
        __enable_irq();

        if (i2c_master_done()) {
            i2c_master_transfer(0x0c, &ui_ocxo_control_word, sizeof(ui_ocxo_control_word));
        }
    }
}
