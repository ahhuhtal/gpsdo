#include <stdio.h>
#include <stdarg.h>
#include <stdbool.h>

#include "i2c.h"
#include "unixtime.h"

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

int putchar_hd44780(int ch) {
    send_hd44780_data(ch);
    Delay_Us(100);
    return ch;
}

int puts_hd44780(char* string) {
    while(*string) {
        putchar_hd44780(*(string++));
    }
    return 0;
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

int printf_hd44780_right_justified(uint8_t pos, uint8_t cols, char* format, ...) {
    va_list args;
    va_start(args, format);

    char string[128];
    int ret = mini_vsnprintf(string, sizeof(string), format, args);
    va_end(args);

    set_pos_hd44780(pos);
    if (ret >= cols) {
        puts_hd44780(string);
    } else {
        for(int col=0;col<cols-ret;col++) {
            putchar_hd44780(' ');
        }
        puts_hd44780(string);
    }

    return ret;
}

int printf_hd44780_left_justified(uint8_t pos, uint8_t cols, char* format, ...) {
    va_list args;
    va_start(args, format);

    char string[128];
    int ret = mini_vsnprintf(string, sizeof(string), format, args);
    va_end(args);

    set_pos_hd44780(pos);
    if (ret >= cols) {
        puts_hd44780(string);
    } else {
        puts_hd44780(string);
        for(int col=0;col<cols-ret;col++) {
            putchar_hd44780(' ');
        }
    }

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
    i2c_init(0x12, 100000UL);

    volatile struct __attribute__((packed)) {
        uint32_t timestamp; // unix time of fix.
        int32_t lat; // fraction of 360 degrees in Q0.31 format. North is positive.
        int32_t lon; // fraction of 360 degrees in Q0.31 format. East is positive.
        uint8_t nsat_fix_valid; // bit 7: validity, bits [6:0] number of satellites
    } iface_data;
    int iface_error;

    // data structure for pulling data from us
    volatile struct __attribute__((packed)) {
        int32_t phase_error_raw; // deviation from UTC second, LSB = 50 ns
        int32_t phase_error_filtered; // Q7.24, LSB ~= 3.0 fs
        int32_t frequecy_error_raw; // OCXO frequency error, LSB = 0.5 Hz
        int32_t frequency_error_filtered; // Q7.24, LSB ~= 60 nHz
        uint32_t time_since_valid; // How long since last control update (i.e. since valid GPS data)
        int16_t ocxo_control_word; // OCXO control word
        uint8_t control_mode; // Which control mode are we in 0=FLL, 1=fast PLL, 2=slow PLL
    } pfd_data;
    int pfd_error;

    volatile uint16_t temperature;
    int temperature_error;

    uint32_t error_count = 0;

    while(1) {
        i2c_master_transfer(0x03, &iface_data, sizeof(iface_data));
        while(!i2c_master_done());
        iface_error = i2c_master_error();

        i2c_master_transfer(0x0d, &temperature, sizeof(temperature));
        while(!i2c_master_done());
        temperature_error = i2c_master_error();

        i2c_master_transfer(0x11, &pfd_data, sizeof(pfd_data));
        while(!i2c_master_done());
        pfd_error = i2c_master_error();

        if (iface_error || temperature_error || pfd_error) {
            error_count++;

            if (error_count > 4) {
                set_pos_hd44780(0x00);
                printf_hd44780("!! I2C ERRORS !!");
                set_pos_hd44780(0x40);
                int line_length = 0;
                if (temperature_error) {
                    puts_hd44780("OCXO ");
                    line_length += 5;
                }
                if (iface_error) {
                    puts_hd44780("GNSS ");
                    line_length += 5;
                }
                if (pfd_error) {
                    puts_hd44780("PFD ");
                    line_length += 4;
                }
                for(;line_length<16;line_length++) {
                    putchar_hd44780(' ');
                }
            }
        } else {
            error_count = 0;

            int ferr = (pfd_data.frequency_error_filtered * 500LL) / 16777216LL;
            int perr = (pfd_data.phase_error_filtered * 50LL) / 16777216LL;
            bool valid = (iface_data.nsat_fix_valid & 0x80) != 0;
            bool fix = (iface_data.nsat_fix_valid & 0x40) != 0;

            int nsat = iface_data.nsat_fix_valid & 0x3f;
 
            int time_since_valid = pfd_data.time_since_valid;

            struct utc_time utc;
            utc_from_unix(iface_data.timestamp, &utc);

            // line 1
            if (!valid) {
                printf_hd44780_left_justified(0x00,16,"! No data: %d", time_since_valid);
            } else if (!fix) {
                printf_hd44780_left_justified(0x00,16,"Acq. sat.: %d", time_since_valid);
            } else if (time_since_valid >= 4UL) {
                printf_hd44780_left_justified(0x00,16,"!! No PPS: %d", time_since_valid);
            } else {
                // cols 0,1,2,3,4,5 are for time
                printf_hd44780_left_justified(0x00,7,"%02d%02d%02d", utc.h, utc.m, utc.s);

                // cols 6,7,8,9,10 are for frequency error
                printf_hd44780_right_justified(0x06,5,"%d", ferr);

                // cols 11,12,13,14,15 are for phase error
                if (pfd_data.control_mode < 1) {
                    printf_hd44780_right_justified(0x0b,5,"FLL");
                } else {
                    printf_hd44780_right_justified(0x0b,5,"%d", perr);
                }
            }

            // line 2
            // cols 0,1 are for number of satellites
            if (!valid) {
                printf_hd44780_left_justified(0x40,2,"--");
            } else {
                printf_hd44780_left_justified(0x40,2,"%d", nsat);
            }

            // cols 2,3,4,5,6 are for OCXO temperature
            printf_hd44780_right_justified(0x42,5,"%d.%01d", temperature/16, (temperature % 16) * 10 / 16);

            // cols 7,8,9,10,11,12,13,14,15 are for control code
            printf_hd44780_right_justified(0x4a,6,"%d", pfd_data.ocxo_control_word);
        }
#if FUNCONF_USE_DEBUGPRINTF
        printf("%lu, %d, %d, %d, %d, %d, %d, %d\n", iface_data.timestamp, (int)iface_data.nsat_fix_valid, (int)pfd_data.control_mode, (int)pfd_data.frequecy_error_raw, (int)pfd_data.frequency_error_filtered, (int)pfd_data.phase_error_raw, (int)pfd_data.phase_error_filtered, (int)pfd_data.ocxo_control_word);
        Delay_Ms(500);
#else
        Delay_Ms(100);
#endif
    }
}
