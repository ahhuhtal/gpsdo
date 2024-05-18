#include <stdio.h>
#include <stdarg.h>
#include <stdbool.h>

#include "ch32v003fun.h"

static inline void set_595_clock(bool val) {
    GPIOA->BSHR = (1<<18) | (val?(1<<2):0);
}

static inline void set_595_data(bool val) {
    GPIOA->BSHR = (1<<17) | (val?(1<<1):0);
}

static inline void set_595_latch(bool val) {
    GPIOC->BSHR = (1<<20) | (val?(1<<4):0);
}

void output_byte_595(uint8_t byte) {
    set_595_clock(false);
    set_595_data(false);

    for(int i=0;i<8;i++) {
        set_595_data( (byte & 0x80) != 0);
        set_595_clock(true);
        set_595_clock(false);
        byte <<= 1;
    }

    set_595_latch(true);
    set_595_latch(false);
}

void send_hd44780_command(uint8_t cmd) {
    output_byte_595( (cmd & 0xf0) | (1<<2) );
    output_byte_595( (cmd & 0xf0) );

    cmd <<= 4;

    output_byte_595( (cmd & 0xf0) | (1<<2) );
    output_byte_595( (cmd & 0xf0) );
}

void send_hd44780_data(uint8_t data) {
    output_byte_595( (data & 0xf0) | (1<<2) | (1<<1) );
    output_byte_595( (data & 0xf0) | (1<<1) );

    data <<= 4;

    output_byte_595( (data & 0xf0) | (1<<2) | (1<<1) );
    output_byte_595( (data & 0xf0) | (1<<1) );
}

void init_hd44780() {
    // send set 4 bit interface comand three times
    // to ensure command is correctly interpreted
    for (int i=0;i<3;i++) {
        output_byte_595( 0x20 | (1<<2) );
        output_byte_595( 0x20 );
        Delay_Us(37); // 37 us max cmd execution time
    }

    // set 4 bit interface, 1/16 duty, 5x8 dots
    send_hd44780_command(0b00101000);
    Delay_Us(37); // 37 us max cmd execution time

    // clear display
    send_hd44780_command(0x01);
    Delay_Us(1520); // 1520 us max cmd execution time

    // cursor home
    send_hd44780_command(0x02);
    Delay_Us(1520); // 1520 us max cmd execution time

    // enable display
    send_hd44780_command(0b00001111);
    Delay_Us(37); // 37 us max cmd execution time
}

int main() {
    SystemInit();

    RCC->APB2PCENR |= RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC;

    // PA1: 595 serial shift data
    GPIOA->CFGLR &= ~(0xf << (4*1));
    GPIOA->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_PP) << (4*1);
    // PA2: 595 serial shift clock
    GPIOA->CFGLR &= ~(0xf << (4*2));
    GPIOA->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_PP) << (4*2);
    // PC4: 595 latch clock
    GPIOC->CFGLR &= ~(0xf << (4*4));
    GPIOC->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_PP) << (4*4);

    init_hd44780();

    while(1) {
        send_hd44780_data('!');
        Delay_Ms(1000);
    }
}
