#include "ch32v003fun.h"
#include "owi.h"

#include <stdio.h>
#include <stdarg.h>
#include <stdbool.h>

#define I2C1_CTLR1_DEFAULT (I2C_CTLR1_PE | I2C_CTLR1_ACK)
#define I2C1_CTLR2_DEFAULT (I2C_CTLR2_ITBUFEN | I2C_CTLR2_ITEVTEN | I2C_CTLR2_ITERREN)

const uint16_t control_default = 55000;

volatile int16_t temperature = 0;

volatile int16_t txtemp;
volatile uint8_t* txtempbuf = (uint8_t*)&txtemp;

volatile int16_t rxctrl;
volatile uint8_t* rxctrlbuf = (uint8_t*)&rxctrl;

volatile uint32_t idx = 0;

void I2C1_EV_IRQHandler(void) __attribute__((interrupt));
void I2C1_EV_IRQHandler(void) {
    uint32_t s1 = I2C1->STAR1;
    uint32_t s2 = I2C1->STAR2;
    (void)s2;

    if (s1 & I2C_STAR1_ADDR) {
        idx = 0;
#ifdef TRACING
        printf("Address matched\n");
        if (s2 & I2C_STAR2_TRA) {
            printf("We're transmitting.\n");
        } else {
            printf("We're receiving\n");
        }
#endif
    }

    if (s1 & I2C_STAR1_RXNE) {
        uint8_t d = I2C1->DATAR;
#ifdef TRACING
        printf("Got byte %d\n", d);
#endif
        if (idx < 2) {
            rxctrlbuf[idx] = d;
        }
        idx++;
        if (idx == 2) {
#ifdef TRACING
            printf("Got control %d\n", rxctrl);
 #endif
            TIM1->CH4CVR = control_default + rxctrl;
            idx = 0;
        }
    }

    if (s1 & I2C_STAR1_TXE) {
#ifdef TRACING
        printf("New byte requested\n");
#endif
        if (idx == 0) {
            txtemp = temperature;
#ifdef TRACING
            printf("Transmitting %d\n", txtemp);
#endif
        }

        I2C1->DATAR = txtempbuf[idx];
        idx = (idx + 1) % 2;
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
#ifdef TRACING
        printf("Last byte. Stop.\n");
#endif
        I2C1->STAR1 = s1 & (~I2C_STAR1_AF);
        idx = 0;
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

int main() {
    SystemInit();

    RCC->APB2PCENR |= RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC | RCC_APB2Periph_TIM1;
    RCC->APB1PCENR |= RCC_APB1Periph_I2C1;

    // PA1: temperature sensor 1-wire
    GPIOA->CFGLR &= ~(0xf << (4*1));
    GPIOA->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_OD) << (4*1);

    // PC4: OCXO trim PWM
    GPIOC->CFGLR &= ~(0xf << (4*4));
    GPIOC->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_PP_AF) << (4*4);

    // PC1, PC2: I2C pins
    GPIOC->CFGLR &= ~(0xf << (4*1));
    GPIOC->CFGLR |= (GPIO_Speed_2MHz | GPIO_CNF_OUT_OD_AF) << (4*1);
    GPIOC->CFGLR &= ~(0xf << (4*2));
    GPIOC->CFGLR |= (GPIO_Speed_2MHz | GPIO_CNF_OUT_OD_AF) << (4*2);

    TIM1->PSC = 0;
    TIM1->CCER |= TIM_CC4E;
    TIM1->CHCTLR2 |= TIM_OC4M_2 | TIM_OC2M_1 | TIM_OC2M_0;
    TIM1->CH4CVR = control_default;
    TIM1->BDTR |= TIM_MOE;
    TIM1->CTLR1 |= TIM_CEN;

    NVIC_EnableIRQ(I2C1_EV_IRQn); // Event interrupt
    NVIC_SetPriority(I2C1_EV_IRQn, 0);
    NVIC_EnableIRQ(I2C1_ER_IRQn); // Error interrupt
    NVIC_SetPriority(I2C1_ER_IRQn, 0);

    I2C1->CKCFGR = FUNCONF_SYSTEM_CORE_CLOCK / (2*100000UL);
    I2C1->CTLR2 = FUNCONF_SYSTEM_CORE_CLOCK / 1000000UL;
    I2C1->OADDR1 = 0x0c;
    I2C1->CTLR1 = I2C1_CTLR1_DEFAULT;
    I2C1->CTLR2 = I2C1_CTLR2_DEFAULT;

    temperature = -32768;

    OWI_address device;
    while(OWI_find_devices(&device, 1) == 0);

    while(1) {
        if(!OWI_busy()) {
            int16_t temp;
            if(OWI_DS18B20readtemp(device, &temp) == 0) {
                temperature = temp;
            }
            OWI_DS18B20setresolution(device, 3);
            OWI_DS18B20startconvert(device);
        }

    }
}
