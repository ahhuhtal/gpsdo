#include <stdio.h>
#include <stdarg.h>
#include <stdbool.h>

#include "ch32v003fun.h"

#include "owi.h"
#include "i2c.h"

const uint16_t control_default = 32768;

volatile int16_t temperature = 0;

// double buffer for temperature to send when pulled
volatile int16_t txtemp;

// double buffer for control to receive when pushed
volatile int16_t rxctrl;

void tx_start_callback(void) {
    txtemp = temperature;
}

void rx_done_callback(void) {
    TIM1->CH4CVR = control_default + rxctrl;
}

int main() {
    SystemInit();

    RCC->APB2PCENR |= RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC | RCC_APB2Periph_TIM1;

    // PA1: temperature sensor 1-wire
    GPIOA->CFGLR &= ~(0xf << (4*1));
    GPIOA->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_OD) << (4*1);

    // PC4: OCXO trim PWM
    GPIOC->CFGLR &= ~(0xf << (4*4));
    GPIOC->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_PP_AF) << (4*4);

    TIM1->PSC = 0;
    TIM1->CCER |= TIM_CC4E;
    TIM1->CHCTLR2 |= TIM_OC4M_2 | TIM_OC2M_1 | TIM_OC2M_0;
    TIM1->CH4CVR = control_default;
    TIM1->BDTR |= TIM_MOE;
    TIM1->CTLR1 |= TIM_CEN;

    i2c_init(0x0c, 100000UL);
    i2c_set_slave_tx(&txtemp, sizeof(txtemp), tx_start_callback);
    i2c_set_slave_rx(&rxctrl, sizeof(rxctrl), rx_done_callback);

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
