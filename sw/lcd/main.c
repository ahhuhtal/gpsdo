#include <stdio.h>
#include <stdarg.h>
#include <stdbool.h>
#include <limits.h>

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

/**
 * History is shown as a graph using 8 characters on the display.
 * Each character has 5 pixel columns, and thus the total history has 40 time intervals.
 * For each interval, the minimum and maximum of recorded parameters is stored.
 * The recorded parameters are the phase error, the frequency error,
 * the OCXO control word and the OCXO temperature.
 *
 * History is kept in two time scales: long and short.
 * In the short scale, the total history is 8 hours. Each character is thus one hour.
 * In the long scale, the total history is 8 days. Each character is thus one day.
 *
 * The history display alternates between the different variables and the time scales.
 */

const unsigned int history_long_length_seconds = 17280; // 4.8 hours of history
const unsigned int history_short_length_seconds = 720; // 12 minutes of history

enum { N_history = 40 }; // number of history intervals

int16_t frequency_history_short_min[N_history];
int16_t frequency_history_short_max[N_history];

int16_t phase_history_short_min[N_history];
int16_t phase_history_short_max[N_history];

int16_t temperature_history_short_min[N_history];
int16_t temperature_history_short_max[N_history];

int16_t satellites_history_short_min[N_history];
int16_t satellites_history_short_max[N_history];

int16_t control_history_short_min[N_history];
int16_t control_history_short_max[N_history];


int16_t frequency_history_long_min[N_history];
int16_t frequency_history_long_max[N_history];

int16_t phase_history_long_min[N_history];
int16_t phase_history_long_max[N_history];

int16_t temperature_history_long_min[N_history];
int16_t temperature_history_long_max[N_history];

int16_t satellites_history_long_min[N_history];
int16_t satellites_history_long_max[N_history];

int16_t control_history_long_min[N_history];
int16_t control_history_long_max[N_history];

// switch display period
const unsigned int display_switch_period_seconds = 10;

int16_t cast_int16_saturate(int value) {
    if (value > INT16_MAX) {
        return INT16_MAX;
    } else if (value < INT16_MIN) {
        return INT16_MIN;
    } else {
        return value;
    }
}

void update_min_max(int16_t* min, int16_t* max, const int16_t value) {
    if (value < *min) {
        *min = value;
    }
    if (value > *max) {
        *max = value;
    }
}

void initialize_min_max(int16_t* min, int16_t* max) {
    *min = INT16_MAX;
    *max = INT16_MIN;
}

void initialize_history(int16_t* history_min, int16_t* history_max) {
    for (int i = 0; i < N_history; i++) {
        initialize_min_max(&history_min[i], &history_max[i]);
    }
}

void roll_history(int16_t* history_min, int16_t* history_max) {
    for (int i = N_history - 1; i > 0; i--) {
        history_min[i] = history_min[i - 1];
        history_max[i] = history_max[i - 1];
    }

    initialize_min_max(&history_min[0], &history_max[0]);
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
        int32_t frequency_error_raw; // Q7.24, LSB ~= 29.8 nHz
        int32_t frequency_error_filtered; // Q7.24, LSB ~= 29.8 nHz
        uint32_t time_since_valid; // How many seconds since last control update (i.e. since valid GPS data)
        int16_t ocxo_control_word; // OCXO control word
        uint8_t control_mode; // Which control mode are we in 0=FLL, 1=PLL
    } pfd_data;
    int pfd_error;

    // temperature fetched from the OCXO board
    volatile uint16_t temperature_data;
    int temperature_error;

    uint32_t error_count = 0;

    enum {
        FREQUENCY_SHORT = 0,
        PHASE_SHORT = 1,
        TEMPERATURE_SHORT = 2,
        SATELLITES_SHORT = 3,
        CONTROL_SHORT = 4,
        FREQUENCY_LONG = 5,
        PHASE_LONG = 6,
        TEMPERATURE_LONG = 7,
        SATELLITES_LONG = 8,
        CONTROL_LONG = 9,
        DISPLAY_END
    } display = FREQUENCY_SHORT;

    // initialize the history arrays
    // short
    initialize_history(frequency_history_short_min, frequency_history_short_max);
    initialize_history(phase_history_short_min, phase_history_short_max);
    initialize_history(temperature_history_short_min, temperature_history_short_max);
    initialize_history(satellites_history_short_min, satellites_history_short_max);
    initialize_history(control_history_short_min, control_history_short_max);

    // long
    initialize_history(frequency_history_long_min, frequency_history_long_max);
    initialize_history(phase_history_long_min, phase_history_long_max);
    initialize_history(temperature_history_long_min, temperature_history_long_max);
    initialize_history(satellites_history_long_min, satellites_history_long_max);
    initialize_history(control_history_long_min, control_history_long_max);

    // time at which to switch to the next short history interval
    uint32_t control_history_short_interval_switch_timestamp = 0;

    // time at which to switch to the next long history interval
    uint32_t control_history_long_interval_switch_timestamp = 0;

    // time at which to switch to the next display
    uint32_t display_switch_timestamp = 0;

    while(1) {
        // fetch data from GNSS interface
        i2c_master_transfer(0x03, &iface_data, sizeof(iface_data));
        while(!i2c_master_done());
        iface_error = i2c_master_error();

        // fetch temperature from OCXO board
        i2c_master_transfer(0x0d, &temperature_data, sizeof(temperature_data));
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
                for (; line_length < 16; line_length++) {
                    putchar_hd44780(' ');
                }
            }
        } else {
            error_count = 0;

            // compute ferr as number of millihertz as Q12.3
            int frequency = (pfd_data.frequency_error_filtered * 500LL) / 2097152LL;

            // compute perr as number of nanoseconds as Q12.3
            int phase = (pfd_data.phase_error_filtered * 50LL) / 2097152LL;

            // compute temperature as Q12.3
            int temperature = temperature_data / 2;

            // control word
            int control_word = pfd_data.ocxo_control_word;

            bool valid = (iface_data.nsat_fix_valid & 0x80) != 0;
            bool fix = (iface_data.nsat_fix_valid & 0x40) != 0;

            int satellites = (valid && fix) ? (iface_data.nsat_fix_valid & 0x3f) : 0;

            int time_since_valid = pfd_data.time_since_valid;

            struct utc_time utc;
            utc_from_unix(iface_data.timestamp, &utc);

            // line 1
            if (!valid) {
                printf_hd44780_left_justified(0x00,16,"! No data: %d", time_since_valid);
            } else if (!fix) {
                printf_hd44780_left_justified(0x00,16,"Acq. sat.: %d", time_since_valid);
            } else if (time_since_valid >= 4) {
                printf_hd44780_left_justified(0x00,16,"!! No PPS: %d", time_since_valid);
            } else {
                // cols 0,1,2,3,4,5 are for time
                printf_hd44780_left_justified(0x00,6,"%02d%02d%02d", utc.h, utc.m, utc.s);

                // cols 6,7,8,9,10,11,12,13,14,15 are for alternating display
                if (display == FREQUENCY_SHORT || display == FREQUENCY_LONG) {
                    printf_hd44780_right_justified(0x06,10,"%dmHz", frequency / 8);
                }
                if (display == PHASE_SHORT || display == PHASE_LONG) {
                    if (pfd_data.control_mode == 1) {
                        // only display phase when in phase lock
                        printf_hd44780_right_justified(0x06,10,"%dns", phase / 8);
                    } else {
                        // otherwise just display a placeholder
                        printf_hd44780_right_justified(0x06,10,"--ns");
                    }
                }
                if (display == TEMPERATURE_SHORT || display == TEMPERATURE_LONG) {
                    if (temperature >= 0) {
                        printf_hd44780_right_justified(0x06,10,"%d.%1dC", temperature / 8, (temperature % 8) * 10 / 8);
                    } else {
                        printf_hd44780_right_justified(0x06,10,"-%d.%1dC", (-temperature) / 8, ((-temperature) % 8) * 10 / 8);
                    }
                }
                if (display == SATELLITES_SHORT || display == SATELLITES_LONG) {
                    printf_hd44780_right_justified(0x06,10,"%d sat", satellites);
                }
                if (display == CONTROL_SHORT || display == CONTROL_LONG) {
                    printf_hd44780_right_justified(0x06,10,"%d #", control_word);
                }
            }

            // line 2 cols 0,1,2 is for lock state
            if (pfd_data.control_mode == 0 || !valid || !fix || time_since_valid >= 4) {
                printf_hd44780_left_justified(0x40,3,"unl");
            } else if (pfd_data.control_mode == 1) {
                printf_hd44780_left_justified(0x40,3,"LCK");
            } else {
                printf_hd44780_left_justified(0x40,3,"???");
            }

            // update the short history
            if (iface_data.timestamp >= control_history_short_interval_switch_timestamp) {
                // switch to next history interval
                control_history_short_interval_switch_timestamp = ((iface_data.timestamp + history_short_length_seconds) / history_short_length_seconds) * history_short_length_seconds;

                // shift the history
                roll_history(phase_history_short_min, phase_history_short_max);
                roll_history(frequency_history_short_min, frequency_history_short_max);
                roll_history(temperature_history_short_min, temperature_history_short_max);
                roll_history(satellites_history_short_min, satellites_history_short_max);
                roll_history(control_history_short_min, control_history_short_max);
            }

            // update the long history
            if (iface_data.timestamp >= control_history_long_interval_switch_timestamp) {
                // switch to next history interval
                control_history_long_interval_switch_timestamp = ((iface_data.timestamp + history_long_length_seconds) / history_long_length_seconds) * history_long_length_seconds;

                // shift the history
                roll_history(phase_history_long_min, phase_history_long_max);
                roll_history(frequency_history_long_min, frequency_history_long_max);
                roll_history(temperature_history_long_min, temperature_history_long_max);
                roll_history(satellites_history_long_min, satellites_history_long_max);
                roll_history(control_history_long_min, control_history_long_max);
            }

            if (iface_data.timestamp >= display_switch_timestamp) {
                // switch to next display
                display_switch_timestamp = iface_data.timestamp + display_switch_period_seconds;

                // switch display
                display++;
                if (display >= DISPLAY_END) {
                    display = FREQUENCY_SHORT;
                }
            }

            if (valid && fix && time_since_valid < 4 && pfd_data.control_mode == 1) {
                // update the short history with the current values
                update_min_max(&frequency_history_short_min[0], &frequency_history_short_max[0], cast_int16_saturate(frequency));
                update_min_max(&phase_history_short_min[0], &phase_history_short_max[0], cast_int16_saturate(phase));
                update_min_max(&temperature_history_short_min[0], &temperature_history_short_max[0], cast_int16_saturate(temperature));
                update_min_max(&satellites_history_short_min[0], &satellites_history_short_max[0], cast_int16_saturate(satellites));
                update_min_max(&control_history_short_min[0], &control_history_short_max[0], cast_int16_saturate(control_word));

                // update the long history with the current values
                update_min_max(&frequency_history_long_min[0], &frequency_history_long_max[0], cast_int16_saturate(frequency));
                update_min_max(&phase_history_long_min[0], &phase_history_long_max[0], cast_int16_saturate(phase));
                update_min_max(&temperature_history_long_min[0], &temperature_history_long_max[0], cast_int16_saturate(temperature));
                update_min_max(&satellites_history_long_min[0], &satellites_history_long_max[0], cast_int16_saturate(satellites));
                update_min_max(&control_history_long_min[0], &control_history_long_max[0], cast_int16_saturate(control_word));
            }

            int history_multiplier = 1; // multiplier to apply to history values before rendering
            int16_t *history_max = frequency_history_short_max;
            int16_t *history_min = frequency_history_short_min;

            if (display == FREQUENCY_SHORT) {
                history_max = frequency_history_short_max;
                history_min = frequency_history_short_min;
                history_multiplier = 1;
            }
            if (display == FREQUENCY_LONG) {
                history_max = frequency_history_long_max;
                history_min = frequency_history_long_min;
                history_multiplier = 1;
            }
            if (display == PHASE_SHORT) {
                history_max = phase_history_short_max;
                history_min = phase_history_short_min;
                history_multiplier = 1;
            }
            if (display == PHASE_LONG) {
                history_max = phase_history_long_max;
                history_min = phase_history_long_min;
                history_multiplier = 1;
            }
            if (display == TEMPERATURE_SHORT) {
                history_max = temperature_history_short_max;
                history_min = temperature_history_short_min;
                history_multiplier = 1;
            }
            if (display == TEMPERATURE_LONG) {
                history_max = temperature_history_long_max;
                history_min = temperature_history_long_min;
                history_multiplier = 1;
            }
            if (display == SATELLITES_SHORT) {
                history_max = satellites_history_short_max;
                history_min = satellites_history_short_min;
                history_multiplier = 8; // set minimum scale to 8 satellites in character
            }
            if (display == SATELLITES_LONG) {
                history_max = satellites_history_long_max;
                history_min = satellites_history_long_min;
                history_multiplier = 8; // set minimum scale to 8 satellites in character
            }
            if (display == CONTROL_SHORT) {
                history_max = control_history_short_max;
                history_min = control_history_short_min;
                history_multiplier = 8; // set minimum scale to 8 counts in character
            }
            if (display == CONTROL_LONG) {
                history_max = control_history_long_max;
                history_min = control_history_long_min;
                history_multiplier = 8; // set minimum scale to 8 counts in character
            }

            // compute the total maximum and minimum of the history

            int history_multiplied_cumulative_max = INT_MIN;
            int history_multiplied_cumulative_min = INT_MAX;

            for (int i_history = 0; i_history < N_history; i_history++) {
                if (history_max[i_history] * history_multiplier > history_multiplied_cumulative_max) {
                    history_multiplied_cumulative_max = history_max[i_history] * history_multiplier;
                }

                if (history_min[i_history] * history_multiplier < history_multiplied_cumulative_min) {
                    history_multiplied_cumulative_min = history_min[i_history] * history_multiplier;
                }
            }

            // number of pixels on the screen
            const int pixel_rows = 8;

            // try to fit entire range on screen
            int history_multiplied_scale = (history_multiplied_cumulative_max - history_multiplied_cumulative_min + pixel_rows - 1) / pixel_rows;

            // clamp scale to reasonable limits
            if (history_multiplied_scale < history_multiplier) {
                history_multiplied_scale = history_multiplier;
            }
            if (history_multiplied_scale > 999) {
                history_multiplied_scale = 999;
            }

            // zero level value (drawn in the middle of the screen)
            const int history_multiplied_display_offset = (history_multiplied_cumulative_max + history_multiplied_cumulative_min) / 2;

            // update the display (custom chars 0 - 7)
            for (int ch = 0; ch < 8; ch++) {
                uint8_t custom_char_data[8];

                for (int row = 0; row < 8; row++) {
                    custom_char_data[row] = 0;
                }

                for (int column = 0; column < 5; column++) {
                    // 5 intervals per character, one pixel column per interval
                    int i_history = ch * 5 + column;

                    if (history_max[i_history] >= history_min[i_history]) {
                        int max_pixel = ((pixel_rows * history_multiplied_scale / 2 - 1) - (history_max[i_history] * history_multiplier - history_multiplied_display_offset)) / history_multiplied_scale;
                        int min_pixel = ((pixel_rows * history_multiplied_scale / 2 - 1) - (history_min[i_history] * history_multiplier - history_multiplied_display_offset)) / history_multiplied_scale;

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
                    }

                    write_cgram_char_hd44780(ch, custom_char_data);
                }
            }

            // line 2 col 3 is empty
            set_pos_hd44780(0x43);
            putchar_hd44780(' ');

            // line 2 cols 4,5,6,7,8,9,10,11 are for displaying history
            set_pos_hd44780(0x44);
            // output the custom characters as 7, 6, 5, 4, 3, 2, 1, 0
            // can't use a string here because we want to print character zero
            for (int ch = 7; ch >= 0; ch--) {
                putchar_hd44780(ch);
            }

            // show history length on column 12
            set_pos_hd44780(0x4c); // column 12
            if (display == FREQUENCY_SHORT || display == PHASE_SHORT || display == TEMPERATURE_SHORT || display == SATELLITES_SHORT || display == CONTROL_SHORT) {
                putchar_hd44780('S'); // short history
            } else {
                putchar_hd44780('L'); // long history
            }

            // show the scale on line columns 13, 14, 15
            printf_hd44780_right_justified(0x4d,3,"%03d", history_multiplied_scale);
        }
#if FUNCONF_USE_DEBUGPRINTF
        printf("%lu, %d, %d, %d, %d, %d, %d, %d\n", iface_data.timestamp, (int)iface_data.nsat_fix_valid, (int)pfd_data.control_mode, (int)pfd_data.frequency_error_raw, (int)pfd_data.frequency_error_filtered, (int)pfd_data.phase_error_raw, (int)pfd_data.phase_error_filtered, (int)pfd_data.ocxo_control_word);
        Delay_Ms(500);
#else
        Delay_Ms(100);
#endif
    }
}
