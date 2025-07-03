#include <stdio.h>
#include <stdarg.h>
#include <stdbool.h>

#include "i2c.h"
#include "unixtime.h"

#include "ch32fun.h"

int mini_vsnprintf(char *buffer, unsigned int buffer_len, const char *fmt, va_list va);

/*
 * GPSDO LCD board sub-unit software implementation
 *
 * 1. Pulls data from the GNSS interface, PLL controller and OCXO board
 * 2. Controls an HD44780 LCD through a 74HC595 shift register
 *    to display the data for the user.
 *
 * The HC595 is connected to the MCU as follows:
 *  PA2 --> SRCLK (shift clock)
 *  PA1 --> SDIN (shift data in)
 *  PC4 --> RCLK (latch clock)
 *
 * The HD44780 is connected to the HC595 as follows:
 *  QB (bit 1) --> RS
 *  QC (bit 2) --> E
 *  QE (bit 4) --> D4
 *  QF (bit 5) --> D5
 *  QG (bit 6) --> D6
 *  QH (bit 7) --> D7
 * Note: QA, QD and QH' are not used.
 */

/**
 * Set HC595 shift clock state
 * @param val true for high, false for low
 */
static inline void set_595_clock(bool val) {
    // bit 18 is PA2 clear, bit 2 is PA2 set
    GPIOA->BSHR = (1 << 18) | (val ? (1 << 2) : 0);
}

/**
 * Set HC595 shift in data
 * @param val true for high, false for low
 */
static inline void set_595_data(bool val) {
    // bit 17 is PA1 clear, bit 1 is PA1 set
    GPIOA->BSHR = (1 << 17) | (val ? (1 << 1) : 0);
}

/**
 * Set HC595 latch clock state
 * @param val true for high, false for low
 */
static inline void set_595_latch(bool val) {
    // bit 20 is PC4 clear, bit 4 is PC4 set
    GPIOC->BSHR = (1 << 20) | (val ? (1 << 4) : 0);
}

/**
 * Shift a byte in the HC595 shift register and latch it out
 * @param byte the byte to output
 */
void output_byte_595(uint8_t byte) {
    set_595_clock(false);
    set_595_data(false);

    // clock the bits in
    for (int i = 0; i < 8; i++) {
        set_595_data( (byte & 0x80) != 0 );
        set_595_clock(true);
        set_595_clock(false);
        byte <<= 1;
    }

    // latch the byte out
    set_595_latch(true);
    set_595_latch(false);
}

/**
 * Send data/command to the HD44780 display.
 * Note: Only the high nibble of the value is used, the low 4 bits are always sent as 0.
 * @param rs true if data, false if command
 * @param value the value to output
 */
void send_hd44780_register(bool rs, uint8_t value) {
    // pick high nibble of value
    // set or clear bit 1 to select the register
    uint8_t byte = (value & 0xf0) | (rs ? (1 << 1) : 0);
    // output byte with E = 1
    output_byte_595(byte | (1 << 2));
    // output byte with E = 0
    output_byte_595(byte);
}

/**
 * Send a command to the HD44780 display in 4 bit mode
 * @param cmd the command to send
 */
void send_hd44780_command(uint8_t cmd) {
    // output bits 7..4 as command
    send_hd44780_register(false, cmd);
    // output bits 3..0 as command
    send_hd44780_register(false, cmd << 4);
}

/**
 * Send a data byte to the HD44780 display in 4 bit mode
 * @param data the data byte to send
 */
void send_hd44780_data(uint8_t data) {
    // output bits 7..4 as data
    send_hd44780_register(true, data);
    // output bits 3..0 as command
    send_hd44780_register(true, data << 4);
}

/**
 * Initialize the HD44780 display.
 *
 * During initialization, the display controller
 * is first put into 8 bit mode and then to 4 bit mode
 */
void init_hd44780() {
    // allow display some time to power on
    Delay_Ms(100);

    /*
     * Set the controller into 8 bit mode.
     * This can be guaranteed from any state by
     * writing the function set command 3 times
     */
    for (int i=0;i<3;i++) {
        // set 8 bit mode
        send_hd44780_register(false, 0x30);
        // wait maximum command execution time
        Delay_Us(4100);
    }

    // the display is now guaranteed to be in 8 bit mode

    // set 4 bit mode
    send_hd44780_register(false, 0x20);
    // wait for maximum command execution time
    Delay_Us(100);

    /*
     * Display is now guaranteed to be in 4 bit mode.
     * Carry on with proper configuration.
     */

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

/**
 * Move cursor to given position
 * @param pos position to move the cursor to
 */
void set_pos_hd44780(uint8_t pos) {
    send_hd44780_command(0x80 | (pos & 0x7f));
    Delay_Us(100);
}

/**
 * Write character data to CGRAM
 * @param ch character code
 * @param data character bitmap data
 */
void write_cgram_char_hd44780(uint8_t ch, uint8_t data[8]) {
    // write the character data
    for (int i = 0; i < 8; i++) {
        // set CGRAM address
        send_hd44780_command(0x40 | (ch << 3) | i);
        Delay_Us(100);
        send_hd44780_data(data[i]);
        Delay_Us(100);
    }
}

/**
 * Output a character to the HD44780 display
 * @param ch character to send
 * @return the character which was output
 */
int putchar_hd44780(int ch) {
    send_hd44780_data(ch);
    Delay_Us(100);
    return ch;
}

/**
 * Output a string to the HD44780 display
 * @param string string to output
 * @return the value 0
 */
int puts_hd44780(char* string) {
    while(*string) {
        putchar_hd44780(*(string++));
    }
    return 0;
}

/**
 * printf implementation for HD44780 display
 * @param format format string
 * @param ... further arguments
 * @return number of chars printed
 */
int printf_hd44780(char* format, ...) {
    va_list args;
    va_start(args, format);

    char string[128];
    int ret = mini_vsnprintf(string, sizeof(string), format, args);
    va_end(args);

    puts_hd44780(string);

    return ret;
}

/**
 * printf to a given field as right justified
 * If string does not fit field, prints starting at field start position
 * and exceeds field width toward the right.
 * @param pos field start position
 * @param cols width of field
 * @param format format string
 * @param ... further arguments
 * @return number of relevant chars printed
 */
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

/**
 * printf to a given field as left justified
 * If string does not fit field, prints starting at field start position
 * and exceeds field width toward the right.
 * @param pos field start position
 * @param cols width of field
 * @param format format string
 * @param ... further arguments
 * @return number of relevant chars printed
 */
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

    // data structure fetched from GNSS interface
    volatile struct __attribute__((packed)) {
        uint32_t timestamp; // unix time of fix.
        int32_t lat; // fraction of 360 degrees in Q0.31 format. North is positive.
        int32_t lon; // fraction of 360 degrees in Q0.31 format. East is positive.
        uint8_t nsat_fix_valid; // bit 7: is valid, bit 6: has fix, bits [5:0] number of satellites
    } iface_data;
    int iface_error;

    // data structure fetched from PLL controller
    volatile struct __attribute__((packed)) {
        int32_t phase_error_raw; // Q7.24, LSB ~= 3.0 fs
        int32_t phase_error_filtered; // Q7.24, LSB ~= 3.0 fs
        int32_t frequecy_error_raw; // Q7.24, LSB ~= 29.8 nHz
        int32_t frequency_error_filtered; // Q7.24, LSB ~= 29.8 nHz
        uint32_t time_since_valid; // How many seconds since last control update (i.e. since valid GPS data)
        int16_t ocxo_control_word; // OCXO control word
        uint8_t control_mode; // Which control mode are we in 0=FLL, 1=fast PLL, 2=slow PLL
    } pfd_data;
    int pfd_error;

    // temperature fetched from the OCXO board
    volatile uint16_t temperature;
    int temperature_error;

    uint32_t error_count = 0;

    // history is shown as a graph using 8 characters on the display
    // each day is split into 5 time intervals, one pixel column per interval
    // for each interval, the minimum and maximum OCXO control word is stored
    // for ease of use, each character represents a single day of history

    const unsigned int control_history_length_seconds = 17280; // 4.8 hours of history

    enum { N_history = 40 }; // number of history intervals
    int16_t control_history_min[N_history];
    int16_t control_history_max[N_history];

    for (unsigned int i = 0; i < N_history; i++) {
        control_history_min[i] = 32767;
        control_history_max[i] = -32768;
    }

    // time at which to switch to the next history interval
    uint32_t control_history_interval_switch_timestamp = 0;

    while(1) {
        // fetch data from GNSS interface
        i2c_master_transfer(0x03, &iface_data, sizeof(iface_data));
        while(!i2c_master_done());
        iface_error = i2c_master_error();

        // fetch temperature from OCXO board
        i2c_master_transfer(0x0d, &temperature, sizeof(temperature));
        while(!i2c_master_done());
        temperature_error = i2c_master_error();

        // fetch data from PLL controller
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

            // compute ferr as number of millihertz
            int ferr = (pfd_data.frequency_error_filtered * 500LL) / 16777216LL;
            // compute perr as number of nanoseconds
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

            if (valid) {
                // update the OCXO control history
                if (iface_data.timestamp >= control_history_interval_switch_timestamp) {
                    // switch to next history interval
                    control_history_interval_switch_timestamp = iface_data.timestamp + control_history_length_seconds;

                    // shift the history
                    for (int i = N_history - 1; i > 0; i--) {
                        control_history_min[i] = control_history_min[i - 1];
                        control_history_max[i] = control_history_max[i - 1];
                    }

                    control_history_min[0] = 32767;
                    control_history_max[0] = -32768;
                }

                if (pfd_data.ocxo_control_word < control_history_min[0]) {
                    control_history_min[0] = pfd_data.ocxo_control_word;
                }
                if (pfd_data.ocxo_control_word > control_history_max[0]) {
                    control_history_max[0] = pfd_data.ocxo_control_word;
                }
            }

            // cols 7,8,9,10,11,12,13,14 are for displaying control history
            // compute the total maximum and minimum of the control history
            int16_t control_history_cumulative_max = -32768;
            int16_t control_history_cumulative_min = 32767;

            for (unsigned int i_history = 0; i_history < N_history; i_history++) {
                if (control_history_max[i_history] > control_history_cumulative_max) {
                    control_history_cumulative_max = control_history_max[i_history];
                }

                if (control_history_min[i_history] < control_history_cumulative_min) {
                    control_history_cumulative_min = control_history_min[i_history];
                }
            }

            // number of pixels on the screen
            const int pixel_rows = 8;

            // scaling factor from control values to number of pixels on screen
            // try to fit entire range on screen
            int history_scale = (control_history_cumulative_max - control_history_cumulative_min + pixel_rows - 1) / pixel_rows;

            // clamp scale to reasonable limits
            if (history_scale < 1) {
                history_scale = 1;
            }
            if (history_scale > 16) {
                history_scale = 16;
            }

            // zero level value (drawn in the middle of the screen)
            int16_t control_history_display_offset;

            if (control_history_cumulative_max - control_history_cumulative_min <= pixel_rows * history_scale) {
                // if the history fits the screen, use the average of the min and max
                control_history_display_offset = (control_history_cumulative_max + control_history_cumulative_min) / 2;
            } else {
                // otherwise always fit the minimum on the screen
                control_history_display_offset = control_history_cumulative_min + pixel_rows * history_scale;
            }

            // update the display (custom chars 0 - 7)
            for (unsigned int ch = 0; ch < 8; ch++) {
                uint8_t custom_char_data[8];

                for (unsigned int row = 0; row < 8; row++) {
                    custom_char_data[row] = 0;
                }

                for (unsigned int column = 0; column < 5; column++) {
                    // 5 intervals per character, one pixel column per interval
                    unsigned int i_history = ch * 5 + column;

                    int max_pixel = ((pixel_rows * history_scale / 2 - 1) - (control_history_max[i_history] - control_history_display_offset)) / history_scale;
                    int min_pixel = ((pixel_rows * history_scale / 2 - 1) - (control_history_min[i_history] - control_history_display_offset)) / history_scale;

                    if (max_pixel < 0) {
                        max_pixel = 0;
                    }
                    if (max_pixel > pixel_rows - 1) {
                        max_pixel = pixel_rows - 1;
                    }
                    if (min_pixel < 0) {
                        min_pixel = 0;
                    }
                    if (min_pixel > pixel_rows - 1) {
                        min_pixel = pixel_rows - 1;
                    }

                    // min is the bottom of the column, max is the top of the column
                    // thus the pixel index for min should be larger than the index for max
                    // draw a line from max to min if they are not reversed.
                    // don't draw a line if they are reversed.
                    for (int row = max_pixel; row <= min_pixel; row++) {
                        custom_char_data[row] |= (1 << column);
                    }

                    write_cgram_char_hd44780(ch, custom_char_data);
                }
            }

            set_pos_hd44780(0x47);
            // output the custom characters as 7, 6, 5, 4, 3, 2, 1, 0
            // can't use a string here because we want to print character zero
            for (int ch = 7; ch >= 0; ch--) {
                putchar_hd44780(ch);
            }

            // show the scale on the last column (15)
            if (history_scale < 10) {
                putchar_hd44780('0' + history_scale - 1);
            } else {
                putchar_hd44780('A' + history_scale - 10);
            }
        }
#if FUNCONF_USE_DEBUGPRINTF
        printf("%lu, %d, %d, %d, %d, %d, %d, %d\n", iface_data.timestamp, (int)iface_data.nsat_fix_valid, (int)pfd_data.control_mode, (int)pfd_data.frequecy_error_raw, (int)pfd_data.frequency_error_filtered, (int)pfd_data.phase_error_raw, (int)pfd_data.phase_error_filtered, (int)pfd_data.ocxo_control_word);
        Delay_Ms(500);
#else
        Delay_Ms(100);
#endif
    }
}
