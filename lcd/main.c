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
    Delay_Ms(100);

    // initially set the controller into 8 bit mode
    // this can be guaranteed from any state by
    // writing the function set command 3 times
    for (int i=0;i<3;i++) {
        output_byte_595( 0x30 | (1<<2) );
        output_byte_595( 0x30 );
        Delay_Us(4100);
    }

    // display is now guaranteed to be in 8 bit mode

    // set display to 4 bit mode
    output_byte_595( 0x20 | (1<<2) );
    output_byte_595( 0x20 );
    Delay_Us(100);

    // display is now guaranteed to be in 4 bit mode.
    // carry on with proper configuration.

    // set 4 bit interface, 1/16 duty, 5x8 dots
    send_hd44780_command(0b00101000);
    Delay_Us(100);

    // enable display
    send_hd44780_command(0b00001100);
    Delay_Us(100);

    // clear display
    send_hd44780_command(0x01);
    Delay_Us(3000);

    // cursor home
    send_hd44780_command(0x02);
    Delay_Us(3000);
}

void set_pos_hd44780(uint8_t pos) {
    send_hd44780_command(0x80 | (pos & 0x7f));
    Delay_Us(100);
}

void puts_hd44780(char* string) {
    while(*string) {
        send_hd44780_data(*string);
        Delay_Us(100);
        string++;
    }
}

int mini_vsnprintf(char *buffer, unsigned int buffer_len, const char *fmt, va_list va);

int printf_hd44780(char* format, ...) {
    va_list args;
    va_start(args, format);

    char string[128];
    int ret = mini_vsnprintf(string, sizeof(string), format, args);
    va_end(args);

    puts_hd44780(string);

    return ret;
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

    int val = 0;
    while(1) {
        set_pos_hd44780(0x00);
        printf_hd44780("First row: %d", val);

        set_pos_hd44780(0x40);
        printf_hd44780("Second row: %d", val);

        Delay_Ms(1000);
        val++;
    }
}
