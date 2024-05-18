#include "ch32v003fun.h"

#include <stdio.h>
#include <stdarg.h>
#include <stdbool.h>

#include <string.h>

#define I2C1_CTLR1_DEFAULT (I2C_CTLR1_PE | I2C_CTLR1_ACK)
#define I2C1_CTLR2_DEFAULT (I2C_CTLR2_ITBUFEN | I2C_CTLR2_ITEVTEN | I2C_CTLR2_ITERREN)

volatile uint64_t tick_high = 0;

volatile uint64_t last_tick_pps;
volatile int last_difftick_pps;
volatile bool last_valid_pps = false;

volatile uint32_t last_timestamp = 0;
volatile uint64_t last_tick_timestamp = 0;
volatile bool last_valid_timestamp = false;

// OCXO control word
volatile int16_t ocxo_control;

volatile struct {
    uint32_t timestamp;
    uint8_t valid;
} rxdata;

volatile uint8_t* rxbuf = (uint8_t*)&rxdata;

volatile int16_t txdata;
volatile uint8_t* txbuf = (uint8_t*)&txdata;

volatile uint32_t idx = 0;

// #define TRACING

void I2C1_EV_IRQHandler(void) __attribute__((interrupt));
void I2C1_EV_IRQHandler(void) {
    uint32_t s1 = I2C1->STAR1;
    uint32_t s2 = I2C1->STAR2;
    
#ifdef TRACING
    if (s2 & I2C_STAR2_MSL) {
        printf("We're a master.\n");
    } else {
        printf("We're a slave.\n");
    }

    if (s2 & I2C_STAR2_TRA) {
        printf("We're transmitting.\n");
    } else {
        printf("We're receiving\n");
    }
#endif

    if (s1 & I2C_STAR1_SB) {
#ifdef TRACING
        printf("Start bit sent\n");
#endif
        I2C1->DATAR = 0x0c;
    }

    if (s1 & I2C_STAR1_ADDR) {
        idx = 0;
#ifdef TRACING
        printf("Address matched\n");
#endif
    }

    if (s1 & I2C_STAR1_RXNE) {
#ifdef TRACING
        printf("Got byte\n");
#endif
        rxbuf[idx] = I2C1->DATAR;
        idx++;
        if (idx == 5) {
            last_timestamp = rxdata.timestamp;
            last_valid_timestamp = (rxdata.valid != 0);
            last_tick_timestamp = tick_high + TIM1->CNT;
            idx = 0;
        }
    }

    if (s1 & I2C_STAR1_TXE) {
#ifdef TRACING
        printf("New byte requested\n");
#endif
        if (idx == 0) {
            txdata = ocxo_control;
        }

        if (s2 & I2C_STAR2_MSL) {
            if (idx == 2) {
                I2C1->CTLR1 = I2C1_CTLR1_DEFAULT | I2C_CTLR1_STOP;
#ifdef TRACING
                printf("All sent. Requesting stop\n");
#endif
            } else {
                I2C1->DATAR = txbuf[idx];
                idx++;
            }
        } else {
            I2C1->DATAR = txbuf[idx];
            idx = (idx + 1) % 2;
        }
    }

    if (s1 & I2C_STAR1_STOPF) {
#ifdef TRACING
        printf("Stop condition\n");
#endif
        idx = 0;
        I2C1->CTLR1 = I2C1_CTLR1_DEFAULT;
    }
}

void I2C1_ER_IRQHandler(void) __attribute__((interrupt));
void I2C1_ER_IRQHandler(void) {
    uint32_t s1 = I2C1->STAR1;
    uint32_t s2 = I2C1->STAR2;

    if (s1 & I2C_STAR1_PECERR) {
#ifdef TRACING
        printf("PEC error\n");
#endif
        I2C1->STAR1 = s1 & (~I2C_STAR1_PECERR);
    }
    if (s1 & I2C_STAR1_OVR) {
#ifdef TRACING
        printf("Overrun error\n");
#endif
        I2C1->STAR1 = s1 & (~I2C_STAR1_OVR);
    }
    if (s1 & I2C_STAR1_AF) {
        if (s2 & I2C_STAR2_MSL) {
#ifdef TRACING
            printf("Not acknowledge. Stopping\n");
#endif
            I2C1->CTLR1 = I2C1_CTLR1_DEFAULT | I2C_CTLR1_STOP;
            I2C1->STAR1 = s1 & (~I2C_STAR1_AF);
        } else {
#ifdef TRACING
            printf("Last byte. Stop.\n");
#endif
            I2C1->STAR1 = s1 & (~I2C_STAR1_AF);
        }
    }
    if (s1 & I2C_STAR1_ARLO) {
#ifdef TRACING
        printf("Arbitration lost\n");
#endif
        I2C1->STAR1 = s1 & (~I2C_STAR1_ARLO);
    }
    if (s1 & I2C_STAR1_BERR) {
#ifdef TRACING
        printf("Bus error\n");
#endif
        I2C1->STAR1 = s1 & (~I2C_STAR1_BERR);
    }
}

void TIM1_UP_IRQHandler(void) __attribute__((interrupt));
void TIM1_UP_IRQHandler(void) {
    TIM1->INTFR &= ~TIM_UIF;
    tick_high+=65536;
}

void TIM1_CC_IRQHandler(void) __attribute__((interrupt));
void TIM1_CC_IRQHandler(void) {
    TIM1->INTFR &= ~TIM_CC4IF;
    if (!(TIM1->INTFR & TIM_UIF)) {
        uint64_t new_tick = tick_high + TIM1->CH4CVR;
        last_difftick_pps = new_tick - last_tick_pps;
        if (last_difftick_pps >= 19999900 && last_difftick_pps <= 20000100) {
            last_valid_pps = true;
        } else {
            last_valid_pps = false;
        }
        last_tick_pps = new_tick;
    }
}

int main() {
    SystemInit();

    RCC->APB2PCENR |= RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC | RCC_APB2Periph_TIM1;
    RCC->APB1PCENR |= RCC_APB1Periph_I2C1;

    // PA2: Supposed to be cleaned PPS output
    GPIOA->CFGLR &= ~(0xf << (4*2));
    GPIOA->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_PP) << (4*2);

    // PC1, PC4: I2C pins
    GPIOC->CFGLR &= ~(0xf << (4*2));
    GPIOC->CFGLR |= (GPIO_Speed_2MHz | GPIO_CNF_OUT_OD_AF) << (4*2);
    GPIOC->CFGLR &= ~(0xf << (4*1));
    GPIOC->CFGLR |= (GPIO_Speed_2MHz | GPIO_CNF_OUT_OD_AF) << (4*1);

    // PC4: GNSS PPS input capture
    GPIOC->CFGLR &= ~(0xf << (4*4));
    GPIOC->CFGLR |= (GPIO_Speed_In | GPIO_CNF_IN_FLOATING) << (4*4);

    NVIC_SetPriority(I2C1_EV_IRQn, 0);
    NVIC_SetPriority(I2C1_ER_IRQn, 0);

    NVIC_EnableIRQ(I2C1_EV_IRQn);
    NVIC_EnableIRQ(I2C1_ER_IRQn);

    I2C1->CKCFGR = FUNCONF_SYSTEM_CORE_CLOCK / (2*100000UL);
    I2C1->CTLR2 = FUNCONF_SYSTEM_CORE_CLOCK / 1000000UL;
    I2C1->OADDR1 = 0x10;
    I2C1->CTLR1 = I2C1_CTLR1_DEFAULT;
    I2C1->CTLR2 = I2C1_CTLR2_DEFAULT;

    TIM1->PSC = 0;
    TIM1->CHCTLR2 |= TIM_CC4S_0;
    TIM1->CCER |= TIM_CC4E;
    TIM1->DMAINTENR |= TIM_UIE | TIM_CC4IE;
    TIM1->CTLR1 |= TIM_CEN;

    NVIC_EnableIRQ(TIM1_UP_IRQn);
    NVIC_EnableIRQ(TIM1_CC_IRQn);

    const float scale = 1.0f / 1024.0f;
    const float ocxo_gain = 3500.0f;

    // FLL variables
    float freq_control = -2048;

    // PLL variables
    int64_t phase_offset = 0;

    enum { FREQ, PHASE } state = FREQ;

    while(1) {
        bool valid_pps;
        uint64_t tick_pps;

        bool valid_timestamp;
        uint64_t tick_timestamp;
        uint32_t timestamp;

        __disable_irq();
        valid_pps = last_valid_pps;

        tick_pps = last_tick_pps;

        if (valid_pps) {
            tick_timestamp = last_tick_timestamp;
            valid_timestamp = last_valid_timestamp;
            timestamp = last_timestamp;

            // invalidate the values for the next cycle
            last_valid_pps = false;
            last_valid_timestamp = false;
        }
        __enable_irq();

        if(valid_pps && valid_timestamp) {
            int64_t pps_timestamp_tick_diff = tick_pps - tick_timestamp;

            if (pps_timestamp_tick_diff > 14000000UL && pps_timestamp_tick_diff < 19000000UL) {
                // the received timestamp is for the previous second
                timestamp++;

                int freq_err = 20000000 - last_difftick_pps;
                int64_t tick_abs = timestamp * 20000000ULL;
                int64_t phase_err = tick_abs - tick_pps - phase_offset;

                if (state == FREQ) {
                    static float freq_err_filt = 0;

                    freq_err_filt += scale*freq_err - scale*freq_err_filt;

                    freq_control += 0.25f*scale*ocxo_gain*freq_err_filt;

                    if (freq_control > 4095) {
                        freq_control = 4095;
                    }
                    if (freq_control < -4096) {
                        freq_control = -4096;
                    }

                    ocxo_control = freq_control;
                    I2C1->CTLR1 = I2C1_CTLR1_DEFAULT | I2C_CTLR1_START;

                    static uint32_t freq_stable_count = 0;

                    if (freq_err_filt > -0.015625f && freq_err_filt < 0.015625f) {
                        freq_stable_count++;
                        if(freq_stable_count > 10) {
                            state = PHASE;
                            freq_stable_count = 0;
                            phase_offset = tick_abs - tick_pps;
                        }
                    } else {
                        freq_stable_count = 0;
                    }
                } else if(state == PHASE) {
                    const float alpha = 3.0f * scale;
                    const float pp = scale;
                    const float ii = scale*scale / 3.0f;

                    static float phase_err_filt = 0;
                    static float phase_err_filt_integral = 0;

                    phase_err_filt_integral += phase_err_filt;

                    phase_err_filt += alpha*phase_err - alpha*phase_err_filt;

                    if (phase_err > 256 || phase_err < -256) {
                        state = FREQ;
                    }

                    float control = freq_control + ocxo_gain*pp*phase_err_filt + ocxo_gain*ii*phase_err_filt_integral;

                    ocxo_control = control;
                    I2C1->CTLR1 = I2C1_CTLR1_DEFAULT | I2C_CTLR1_START;
                }

                printf("%d, %lu, %d, %d\n", state, timestamp, (int)freq_err, (int)phase_err, (int)freq_control);
            }
        }
    }
}
