#include <stdio.h>
#include <stdarg.h>
#include <stdbool.h>

#include "ch32v003fun.h"

#include "owi.h"
#include "i2c.h"

const uint16_t control_default = 32768; // control zero value

volatile int16_t temperature; // temperature value to transmit on the next I2C read


/**
 * I2C slave transmit
 */

volatile int16_t tx_buf; // I2C slave tx buffer

// I2C transmit callback, called right before data transmission starts
void tx_start_callback(void) {
    // copy temperature value over to tx buffer
    tx_buf = temperature;
}

/**
 * I2C slave receive
 */
volatile int16_t rx_buf; // I2C slave receiver buffer

// I2C receive callback, called right after data receive complete
void rx_done_callback(void) {
    // set the control output duty cycle based on the received value
    TIM1->CH4CVR = control_default + rx_buf;
}

int main() {
    SystemInit();

    RCC->APB2PCENR |= RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC | RCC_APB2Periph_TIM1;

    // PA1: temperature sensor 1-wire
    GPIOA->CFGLR &= ~(0xf << (4 * 1));
    GPIOA->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_OD) << (4 * 1);

    // PC4: OCXO control voltage PWM
    GPIOC->CFGLR &= ~(0xf << (4 * 4));
    GPIOC->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_PP_AF) << (4 * 4);

    // TIM1 CC4 used for control voltage PWM
    TIM1->PSC = 0; // no prescale
    TIM1->CCER |= TIM_CC4E; // CC4 enabled
    TIM1->CHCTLR2 |= TIM_OC4M_2 | TIM_OC2M_1 | TIM_OC2M_0; // CC4 for PWM out
    TIM1->CH4CVR = control_default;
    TIM1->BDTR |= TIM_MOE; // main output enable
    TIM1->CTLR1 |= TIM_CEN; // counter enable

    i2c_init(0x0c, 100000UL);
    i2c_set_slave_tx(&tx_buf, sizeof(tx_buf), tx_start_callback);
    i2c_set_slave_rx(&rx_buf, sizeof(rx_buf), rx_done_callback);

    // search for a single device on the bus.
    // TODO: use generic addressing instead of searching for the single device
    OWI_address device;
    while(OWI_find_devices(&device, 1) == 0);

    while(1) {
        if (!OWI_busy()) {
            // conversion not running. read previous result.
            int16_t temp;
            if(OWI_DS18B20readtemp(device, &temp) == 0) {
                temperature = temp;
            }

            // start new conversion
            OWI_DS18B20setresolution(device, 3);
            OWI_DS18B20startconvert(device);
        }
    }
}
