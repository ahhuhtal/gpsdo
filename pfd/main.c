#include <stdio.h>
#include <stdarg.h>
#include <stdbool.h>

#include <string.h>

#include "ch32v003fun.h"

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
 * The PLL controller here is a single parameter PI controller
 * with error signal low-pass filtering.
 * See https://kiedontaa.blogspot.com/2024/07/single-parameter-controller-for-gps.html
 *
 * The FLL controller follows the same idea
 */

// tick counter high bits [63:16], bits 15:0 always 0.
volatile uint64_t tick_high = 0;

// tick value of last PPS pulse
volatile uint64_t last_tick_pps;
// tick difference between two last PPS pulses
volatile int last_difftick_pps;
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
volatile int32_t last_phase_error_raw; // raw phase error, 50 ns res.
volatile int32_t last_phase_error_filtered; // filtered phase error, 3.0 fs res.
volatile int32_t last_frequency_error_raw; // raw frequency error, 0.5 Hz res.
volatile int32_t last_frequency_error_filtered; // filtered phase error, 29.8 nHz res.
volatile uint64_t last_tick_valid; // tick of latest OCXO control update
volatile uint16_t last_ocxo_control_word; // last OCXO control word
volatile uint8_t last_control_mode; // controller mode at last update, 0=FLL, 1=fast PLL, 2=slow PLL

// data structure for pulling data from us
volatile struct {
    int32_t phase_error_raw; // deviation from UTC second, LSB = 50 ns
    int32_t phase_error_filtered; // Q7.24, LSB ~= 3.0 fs
    int32_t frequecy_error_raw; // OCXO frequency error, LSB = 0.5 Hz
    int32_t frequency_error_filtered; // Q7.24, LSB ~= 29.8 nHz
    uint32_t time_since_valid; // How many seconds since last control update (i.e. since valid GPS data)
    int16_t ocxo_control_word; // OCXO control word
    uint8_t control_mode; // Which control mode are we in 0=FLL, 1=fast PLL, 2=slow PLL
} __attribute__((packed)) tx_data_buf;

// called when someone wants to pull data from us
void tx_start_callback(void) {
    // double buffer the data so that it changes atomically
    tx_data_buf.phase_error_raw = last_phase_error_raw;
    tx_data_buf.phase_error_filtered = last_phase_error_filtered;
    tx_data_buf.frequecy_error_raw = last_frequency_error_raw;
    tx_data_buf.frequency_error_filtered = last_frequency_error_filtered;
    tx_data_buf.time_since_valid = (get_current_tick() - last_tick_valid) / 20000000UL;
    tx_data_buf.ocxo_control_word = last_ocxo_control_word;
    tx_data_buf.control_mode = last_control_mode;
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
        last_difftick_pps = new_tick - last_tick_pps;
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
 * Whether to build a normal binary CALIB = 0 or a calibration binary CALIB != 0
 * A calibration binary will not run the controller, but instead
 * alternate between two control words, while measuring the frequency
 * This is useful for OCXO gain measurement.
 */
#define CALIB 0

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

    int16_t ocxo_control_word;
    int64_t phase_offset = 0;

    float freq_err_filt = 0;
    float phase_err_filt = 0;

#if CALIB
    const int16_t control_0 = 18438 - 500;
    const int16_t control_1 = 18438 + 500;

    const float alpha = 1.0f / 256.0f;
#else
    /**
     * OCXO control gain.
     * The OCXO response is of the form:
     * frequency_ocxo = control_word / ocxo_gain + frequency_offset
     */
    const float ocxo_gain = 2000.0f;

    const float time_scale_fll = 1.0f / 32.0f;
    const float time_scale_pll_fast = 1.0f / 128.0f;
    const float time_scale_pll_slow = 1.0f / 2048.0f;

    const float alpha_fll = 2.0f * time_scale_fll;
    const float i_fll = ocxo_gain * time_scale_fll / 2.0f;

    const float alpha_pll_fast = 3.0f * time_scale_pll_fast;
    const float p_pll_fast = ocxo_gain * time_scale_pll_fast;
    const float i_pll_fast = ocxo_gain * time_scale_pll_fast * time_scale_pll_fast / 3.0f;

    const float alpha_pll_slow = 3.0f * time_scale_pll_slow;
    const float p_pll_slow = ocxo_gain * time_scale_pll_slow;
    const float i_pll_slow = ocxo_gain * time_scale_pll_slow * time_scale_pll_slow / 3.0f;

    float freq_err_filt_integral = 0.0f / i_fll;
    float freq_err_filt_integral_err = 0; // Kahan-Babushka error

    float phase_err_filt_integral = 0;
    float phase_err_filt_integral_err = 0; // Kahan-Babushka error
#endif
    uint8_t state = 0; // 0=FLL, 1=fast PLL, 2=slow PLL, 255=CALIB
    uint32_t state_count = 0; // time to transition to next state

    while(1) {
        bool valid_pps;
        uint64_t tick_pps;

        bool valid_timestamp;
        uint64_t tick_timestamp;
        uint32_t timestamp;

        __disable_irq(); // read the following data atomically
        valid_pps = last_valid_pps;

        tick_pps = last_tick_pps;

        if (valid_pps) {
            tick_timestamp = last_timestamp_tick;
            valid_timestamp = last_timestamp_valid;
            timestamp = last_timestamp;

            // invalidate the values for the next cycle
            last_valid_pps = false;
            last_timestamp_valid = false;
        }
        __enable_irq();

        if(valid_pps && valid_timestamp) {
            int64_t pps_timestamp_tick_diff = tick_pps - tick_timestamp;

            if (pps_timestamp_tick_diff > 10000000UL && pps_timestamp_tick_diff < 20000000UL) {
                // the received timestamp is for the previous second
                timestamp++;

                int64_t tick_abs = timestamp * 20000000LL;

                int freq_err = 20000000 - last_difftick_pps;
                int phase_err = tick_abs - tick_pps - phase_offset;

#if CALIB
                state = 255;
                freq_err_filt += alpha*(freq_err - freq_err_filt);

                phase_err = 0;
                phase_err_filt = 0;

                if ((state_count % 2048) < 1024) {
                    ocxo_control_word = control_0;
                } else {
                    ocxo_control_word = control_1;
                }

                state_count++;
#else
                if (state == 0) {
                    // Do Kahan-Babushka for: freq_err_filt_integral += freq_err_filt;
                    float freq_err_filt_corrected = freq_err_filt - freq_err_filt_integral_err;
                    float new_freq_err_filt_integral = freq_err_filt_integral + freq_err_filt_corrected;
                    freq_err_filt_integral_err = (new_freq_err_filt_integral - freq_err_filt_integral) - freq_err_filt_corrected;
                    freq_err_filt_integral = new_freq_err_filt_integral;

                    freq_err_filt += alpha_fll * (freq_err - freq_err_filt);

                    float control = i_fll * freq_err_filt_integral;

                    if (control > 32767.0f) {
                        control = 32767.0f;
                    }
                    if (control < -32768.0f) {
                        control = -32768.0f;
                    }

                    ocxo_control_word = control;

                    state_count++;
                    if(state_count >= 512) {
                        state = 1;
                        state_count = 0;

                        // seamless hand over to fast PLL
                        phase_err_filt_integral = control / i_pll_fast;
                        phase_err_filt_integral_err = freq_err_filt_integral_err * i_fll / i_pll_fast;
                    }

                    // constantly reset phase error in FLL mode
                    phase_offset = tick_abs - tick_pps;
                    phase_err = 0;
                    phase_err_filt = 0;
                } else if (state == 1) {
                    freq_err_filt += alpha_pll_fast * (freq_err - freq_err_filt);

                    // Do Kahan-Babushka for: phase_err_filt_integral += phase_err_filt;
                    float phase_err_filt_corrected = phase_err_filt - phase_err_filt_integral_err;
                    float new_phase_err_filt_integral = phase_err_filt_integral + phase_err_filt_corrected;
                    phase_err_filt_integral_err = (new_phase_err_filt_integral - phase_err_filt_integral) - phase_err_filt_corrected;
                    phase_err_filt_integral = new_phase_err_filt_integral;

                    phase_err_filt += alpha_pll_fast * (phase_err - phase_err_filt);

                    float control = p_pll_fast * phase_err_filt + i_pll_fast * phase_err_filt_integral;

                    if (control > 32767.0f) {
                        control = 32767.0f;
                    }
                    if (control < -32768.0f) {
                        control = -32768.0f;
                    }

                    ocxo_control_word = control;

                    state_count++;
                    if(state_count >= 2048) {
                        state = 2;
                        state_count = 0;

                        // seamless hand over to slow PLL
                        phase_err_filt_integral = (control - p_pll_slow * phase_err_filt) / i_pll_slow;
                        phase_err_filt_integral_err = phase_err_filt_integral_err * i_pll_fast / i_pll_slow;
                    }
                } else {
                    freq_err_filt += alpha_pll_slow * (freq_err - freq_err_filt);

                    // Do Kahan-Babushka for: phase_err_filt_integral += phase_err_filt;
                    float phase_err_filt_corrected = phase_err_filt - phase_err_filt_integral_err;
                    float new_phase_err_filt_integral = phase_err_filt_integral + phase_err_filt_corrected;
                    phase_err_filt_integral_err = (new_phase_err_filt_integral - phase_err_filt_integral) - phase_err_filt_corrected;
                    phase_err_filt_integral = new_phase_err_filt_integral;

                    phase_err_filt += alpha_pll_slow * (phase_err - phase_err_filt);

                    if (phase_err_filt > 127.0f || phase_err_filt < -128.0f) {
                        state = 0;
                    }

                    float control = p_pll_slow * phase_err_filt + i_pll_slow * phase_err_filt_integral;

                    if (control > 32767.0f) {
                        control = 32767.0f;
                    }
                    if (control < -32768.0f) {
                        control = -32768.0f;
                    }

                    ocxo_control_word = control;
                }
#endif

                i2c_acquire(); // don't allow i2c accesses while we update the following data
                last_frequency_error_raw = freq_err;
                last_frequency_error_filtered = freq_err_filt * 16777216.0f;

                last_phase_error_raw = phase_err;
                last_phase_error_filtered = phase_err_filt * 16777216.0f;

                last_tick_valid = tick_pps;

                last_ocxo_control_word = ocxo_control_word;

                last_control_mode = state;
                i2c_release();

                __disable_irq();
                next_pps_out_tick = tick_abs - phase_offset + 20000000;
                __enable_irq();

                if (i2c_master_done()) {
                    i2c_master_transfer(0x0c, &last_ocxo_control_word, sizeof(last_ocxo_control_word));
                }

#if (FUNCONF_USE_DEBUGPRINTF)
                int freq_err_filt_scale = last_frequency_error_filtered * 1000LL / 16777216LL;
                int phase_err_filt_scale = last_phase_error_filtered * 1000LL / 16777216LL;

                printf("%d, %lu, %d, %d, %d, %d, %d\n", state, timestamp, (int)last_frequency_error_raw, (int)freq_err_filt_scale, (int)last_phase_error_raw, (int)phase_err_filt_scale, (int)ocxo_control_word);
#endif
            }
        }
    }
}
