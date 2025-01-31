#include <stdio.h>
#include <stdarg.h>
#include <stdbool.h>

#include <string.h>

#include "ch32v003fun.h"

#include "i2c.h"

// tick counter high bits [63:16]
volatile uint64_t tick_high = 0;

// tick value of last PPS pulse
volatile uint64_t last_tick_pps;
// tick difference between two last PPS pulses
volatile int last_difftick_pps;
// was the last PPS pulse valid
volatile bool last_valid_pps = false;

// last received timestamp value
volatile uint32_t last_timestamp = 0;
// tick value of last received timestamp
volatile uint64_t last_timestamp_tick = 0;
// was the last received timestamp valid
volatile bool last_timestamp_valid = false;

// data structure for pushing data to us
volatile struct {
    uint32_t timestamp;
    uint8_t valid;
} __attribute__((packed)) rx_data_buf;

// called when someone has pushed data to us
void rx_done_callback(void) {
    last_timestamp_tick = tick_high + TIM1->CNT;
    // double buffer the data so that it changes atomically
    last_timestamp = rx_data_buf.timestamp;
    last_timestamp_valid = rx_data_buf.valid;
}

volatile int32_t last_phase_error_raw;
volatile int32_t last_phase_error_filtered;
volatile int32_t last_frequency_error_raw;
volatile int32_t last_frequency_error_filtered;
volatile uint64_t last_tick_valid;
volatile uint16_t last_ocxo_control_word;
volatile uint8_t last_control_mode;

// data structure for pulling data from us
volatile struct {
    int32_t phase_error_raw; // deviation from UTC second, LSB = 50 ns
    int32_t phase_error_filtered; // Q7.24, LSB ~= 3.0 fs
    int32_t frequecy_error_raw; // OCXO frequency error, LSB = 0.5 Hz
    int32_t frequency_error_filtered; // Q7.24, LSB ~= 60 nHz
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
    tx_data_buf.time_since_valid = (tick_high + TIM1->CNT - last_tick_valid) / 20000000UL;
    tx_data_buf.ocxo_control_word = last_ocxo_control_word;
    tx_data_buf.control_mode = last_control_mode;
}

volatile uint64_t next_pps;

void TIM1_UP_IRQHandler(void) __attribute__((interrupt));
void TIM1_UP_IRQHandler(void) {
    tick_high += 65536;

    if (next_pps < tick_high) {
        next_pps += 20000000ULL;
    }

    if (next_pps >= tick_high + 32768 && next_pps < tick_high + 65536) {
        // a pps will occur during the latter half of this wraparound period

        TIM1->CH2CVR = next_pps - tick_high;
        TIM1->CHCTLR1 = TIM_OC2M_0; // set output when match
    }
    TIM1->INTFR = ~TIM_UIF;
}

void TIM1_CC_IRQHandler(void) __attribute__((interrupt));
void TIM1_CC_IRQHandler(void) {
    if (TIM1->INTFR & TIM_CC4IF) {
        uint64_t new_tick = tick_high + TIM1->CH4CVR;
        last_difftick_pps = new_tick - last_tick_pps;
        if (last_difftick_pps >= 19999900 && last_difftick_pps <= 20000100) {
            last_valid_pps = true;
        } else {
            last_valid_pps = false;
        }
        last_tick_pps = new_tick;
        TIM1->INTFR = ~TIM_CC4IF;
    }

    if (TIM1->INTFR & TIM_CC2IF) {
        TIM1->CHCTLR1 = TIM_OC2M_1; // clear output when match
        TIM1->INTFR = ~TIM_CC2IF;
    }

    if (TIM1->INTFR & TIM_CC1IF) {
        if (next_pps >= tick_high + 65536 && next_pps < tick_high + 98304) {
            // a pps will occur during the first half of the next wraparound period

            TIM1->CH2CVR = next_pps - (tick_high + 65536);
            TIM1->CHCTLR1 = TIM_OC2M_0; // set output when match
        }
        TIM1->INTFR = ~TIM_CC1IF;
    }
}

#define CALIB 0

int main() {
    SystemInit();

    RCC->APB2PCENR |= RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC | RCC_APB2Periph_TIM1;
    RCC->APB1PCENR |= RCC_APB1Periph_I2C1;

    // PA2: Cleaned PPS output
    GPIOA->CFGLR &= ~(0xf << (4*2));
    GPIOA->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_PP_AF) << (4*2);

    // PC4: GNSS PPS input capture
    GPIOC->CFGLR &= ~(0xf << (4*4));
    GPIOC->CFGLR |= (GPIO_Speed_In | GPIO_CNF_IN_FLOATING) << (4*4);

    i2c_init(0x10, 100000UL);
    i2c_set_slave_rx(&rx_data_buf, sizeof(rx_data_buf), rx_done_callback);
    i2c_set_slave_tx(&tx_data_buf, sizeof(tx_data_buf), tx_start_callback);

    TIM1->PSC = 0;
    TIM1->CHCTLR1 = 0;
    TIM1->CHCTLR2 = TIM_CC4S_0;
    TIM1->CCER = TIM_CC4E | TIM_CC2NE;
    TIM1->CH1CVR = 32768;
    TIM1->DMAINTENR = TIM_UIE | TIM_CC4IE | TIM_CC2IE | TIM_CC1IE;
    TIM1->BDTR = TIM_MOE;
    TIM1->CTLR1 = TIM_CEN;

    NVIC_EnableIRQ(TIM1_UP_IRQn);
    NVIC_SetPriority(TIM1_UP_IRQn, 0xc0); // updates at high priority

    NVIC_EnableIRQ(TIM1_CC_IRQn);

    int16_t ocxo_control_word;
    int64_t phase_offset = 0;

    float freq_err_filt = 0;
    float phase_err_filt = 0;

#if CALIB
    const int16_t control_0 = 18438 - 500;
    const int16_t control_1 = 18438 + 500;

    const float alpha = 1.0f / 256.0f;
#else
    const float ocxo_gain = 2000.0f;

    const float time_scale_fll = 1.0f / 32.0f;
    const float time_scale_pll_fast = 1.0f / 128.0f;
    const float time_scale_pll_slow = 1.0f / 2048.0f;

    const float alpha_fll = 2.0f*time_scale_fll;
    const float i_fll = ocxo_gain*time_scale_fll/2.0f;

    const float alpha_pll_fast = 3.0f*time_scale_pll_fast;
    const float p_pll_fast = ocxo_gain*time_scale_pll_fast;
    const float i_pll_fast = ocxo_gain*time_scale_pll_fast*time_scale_pll_fast/3.0f;

    const float alpha_pll_slow = 3.0f*time_scale_pll_slow;
    const float p_pll_slow = ocxo_gain*time_scale_pll_slow;
    const float i_pll_slow = ocxo_gain*time_scale_pll_slow*time_scale_pll_slow/3.0f;

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

                    freq_err_filt += alpha_fll*(freq_err - freq_err_filt);

                    float control = i_fll*freq_err_filt_integral;

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
                    freq_err_filt += alpha_pll_fast*(freq_err - freq_err_filt);

                    // Do Kahan-Babushka for: phase_err_filt_integral += phase_err_filt;
                    float phase_err_filt_corrected = phase_err_filt - phase_err_filt_integral_err;
                    float new_phase_err_filt_integral = phase_err_filt_integral + phase_err_filt_corrected;
                    phase_err_filt_integral_err = (new_phase_err_filt_integral - phase_err_filt_integral) - phase_err_filt_corrected;
                    phase_err_filt_integral = new_phase_err_filt_integral;

                    phase_err_filt += alpha_pll_fast*(phase_err - phase_err_filt);

                    float control = p_pll_fast*phase_err_filt + i_pll_fast*phase_err_filt_integral;

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
                    freq_err_filt += alpha_pll_slow*(freq_err - freq_err_filt);

                    // Do Kahan-Babushka for: phase_err_filt_integral += phase_err_filt;
                    float phase_err_filt_corrected = phase_err_filt - phase_err_filt_integral_err;
                    float new_phase_err_filt_integral = phase_err_filt_integral + phase_err_filt_corrected;
                    phase_err_filt_integral_err = (new_phase_err_filt_integral - phase_err_filt_integral) - phase_err_filt_corrected;
                    phase_err_filt_integral = new_phase_err_filt_integral;

                    phase_err_filt += alpha_pll_slow*(phase_err - phase_err_filt);

                    if (phase_err_filt > 127.0f || phase_err_filt < -128.0f) {
                        state = 0;
                    }

                    float control = p_pll_slow*phase_err_filt + i_pll_slow*phase_err_filt_integral;

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
                next_pps = tick_abs - phase_offset + 20000000;
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
