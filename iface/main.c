#include <stdio.h>
#include <stdarg.h>
#include <stdbool.h>

#include <string.h>

#include "unixtime.h"
#include "i2c.h"

#include "ch32fun.h"

/**
 * GPSDO GNSS data interface implementation
 *
 * 1. Receives and parses NMEA data from a GNSS receiver
 * 2. Transmits relevant data to the PFD controller via I2C
 * 3. Allows LCD board sub-unit to query GNSS data
 *
 * Due to limitations of the 8 pin packages, UART hardware tx cannot be used.
 * TX for configuration is handled via bit-banging. RX is handled through HW.
 */

// bitrate for NMEA communication
#define NMEA_BITRATE 9600UL

volatile bool valid; // is timestamp and auxiliary data still valid
volatile uint32_t valid_tick; // systick value at latest update

// timestamp data
volatile bool fix; // did we have GNSS fix at latest timestamp?
volatile uint32_t timestamp; // latest recorded timestamp value

// auxiliary data
volatile int32_t latitude; // fraction of 360 degrees in Q0.31 format. North is positive.
volatile int32_t longitude; // fraction of 360 degrees in Q0.31 format. East is positive.
volatile uint8_t n_satellites; // number of satellites

/**
 * Update timestamp
 * @param has_fix Do we have GNSS fix
 * @param Y Year of timestamp (full year)
 * @param M Month of timestamp (1 through 12)
 * @param D Day of timestamp (1 through 31)
 * @param h Hour of timestamp (0 through 23)
 * @param m Minute of timestamp (0 through 59)
 * @param s Second of timestamp (0 through 59)
 */
void update_timestamp(bool has_fix, int Y, int M, int D, int h, int m, int s) {
    uint32_t value = unix_from_utc((struct utc_time){Y, M, D, h, m, s});
    __disable_irq();
    valid_tick = SysTick->CNT;
    fix = has_fix;
    valid = true;
    timestamp = value;
    __enable_irq();
}

/**
 * Update auxiliary data
 * @param lat Latitude fraction of 360 degrees in Q0.31 format. North is positive.
 * @param lon Longitude fraction of 360 degrees in Q0.31 format. East is positive.
 * @param nsat Number of satellites
 */
void update_auxiliary(int32_t lat, int32_t lon, uint8_t nsat) {
    __disable_irq();
    valid_tick = SysTick->CNT;
    latitude = lat;
    longitude = lon;
    n_satellites = nsat;
    valid = true;
    __enable_irq();
}

/**
 * Poll the validity of the data and mark invalid when validity times out
 */
void poll_data() {
    // 4 second timeout
    const uint32_t timeout = 4 * (FUNCONF_SYSTEM_CORE_CLOCK / 8);

    if (SysTick->CNT - valid_tick > timeout) {
        valid = false;
    }
}

/**
 * I2C slave transmit
 */

// packed struct when pulling data
volatile struct __attribute__((packed)) {
    uint32_t timestamp; // unix time of fix.
    int32_t lat; // fraction of 360 degrees in Q0.31 format. North is positive.
    int32_t lon; // fraction of 360 degrees in Q0.31 format. East is positive.
    uint8_t nsat_fix_valid; // bit 7: validity, bit 6: fix, bits [5:0] number of satellites
} tx_buf;

// called just before data transmit starts
void update_tx_callback(void) {
    tx_buf.timestamp = timestamp;
    tx_buf.lat = latitude;
    tx_buf.lon = longitude;
    tx_buf.nsat_fix_valid = (n_satellites & 0x3f) | (fix ? 0x40 : 0x00) | (valid ? 0x80 : 0x00);
}

/**
 * Transmit UART byte by bit-banging
 * @param ch Byte to transmit
 */
void UART_bitbang_tx(uint8_t ch) {
    // construct UART bit pattern:
    // start bit (high) + 8 bit data + stop bit (low)
    uint16_t val = 0x200 | (ch << 1);

    for(int i = 0; i < 10; i++) {
        // set or clear PA2 based on the bit pattern
        GPIOA->BSHR = 0x40000 | (val & 0x01) << 2;
        // wait for bit period
        DelaySysTick(FUNCONF_SYSTEM_CORE_CLOCK / (NMEA_BITRATE * 8));
        val >>= 1;
    }
}
/**
 * Transmit given data over UART
 * @param str Pointer to data to transmit
 * @param len Length of string
 */
void UART_put(char* str, size_t len) {
    for(size_t i = 0; i < len; i++) {
        UART_bitbang_tx(str[i]);
    }
}

/**
 * Transmit given null-terminated string over UART
 * @param str Pointer to string data
 */
void UART_puts(char* str) {
    while(*str) {
        UART_bitbang_tx(*str);
        str++;
    }
}

/**
 * Decode value of given digit character
 * @param ch Character to decode ('0' through '9' and 'A'/'a' through 'F'/'f')
 * @return Value of digit or 0xff if not a valid digit
 */
uint8_t digitvalue(char ch) {
    if (ch >= '0' && ch <= '9') {
        return ch - '0';
    }

    if (ch >= 'A' && ch <= 'F') {
        return ch - 'A' + 0x0a;
    }

    if (ch >= 'a' && ch <= 'f') {
        return ch - 'a' + 0x0a;
    }

    return 0xff;
}

/**
 * Decode exactly two hex digits as an 8 bit value
 * @param hex Pointer to first digit
 * @return Decoded value
 */
uint8_t hexpair_to_value(char* hex) {
    return (digitvalue(hex[0]) << 4) | digitvalue(hex[1]);
}

/**
 * Decode exactly two decimal digits as an 8 bit value
 * @param dec Pointer to first digit
 * @return Decoded value
 */
uint8_t decpair_to_value(char* dec) {
    return digitvalue(dec[0]) * 10 + digitvalue(dec[1]);
}

/**
 * Decode exactly three decimal digits as an 8 bit value
 * Note: it is possible that the value exceeds the range of an uint8_t,
 * in which case a wrap around occurs.
 * @param dec Pointer to first digit
 * @return Decoded value
 */
uint8_t dectriple_to_value(char* dec) {
    return digitvalue(dec[0]) * 100 + digitvalue(dec[1]) * 10 + digitvalue(dec[2]);
}

int main() {
    SystemInit();

    RCC->APB2PCENR |= RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD | RCC_APB2Periph_USART1;
    RCC->APB1PCENR |= RCC_APB1Periph_I2C1;

    // PA2: SW bit-band UART TX
    GPIOA->CFGLR &= ~(0xf << (4 * 2));
    GPIOA->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_PP) << (4 * 2);
    GPIOA->BSHR = 1 << 2;

    // PD6: USART1 RX
    GPIOC->CFGLR &= ~(0xf << (4 * 6));
    GPIOC->CFGLR |= (GPIO_Speed_In | GPIO_CNF_IN_FLOATING) << (4 * 6);

    i2c_init(0x03, 100000UL);
    i2c_set_slave_tx(&tx_buf, sizeof(tx_buf), update_tx_callback);

    // configure UART RX
    USART1->CTLR1 = USART_WordLength_8b | USART_Parity_No | USART_Mode_Rx;
    USART1->CTLR2 = USART_StopBits_1;
    USART1->CTLR3 = USART_HardwareFlowControl_None;
    USART1->BRR = FUNCONF_SYSTEM_CORE_CLOCK  / NMEA_BITRATE;
    USART1->CTLR1 |= CTLR1_UE_Set;

    // buffer for a NMEA message
    char nmeamsg[128];
    size_t nmeamsg_len = 0;

    uint8_t nmeamsg_checksum_calc = 0;
    bool nmeamsg_data_end = false;

    // buffer for transmitting data to PFD controller
    volatile struct __attribute__((packed)) {
        uint32_t timestamp;
        uint8_t fix;
    } push_data_buf;

    // Config GNSS receiver to transmit less data
    Delay_Ms(1000); // wait for receiver bootup
    UART_puts("$PCAS03,1,0,0,0,1,0,0,0,,,,,,*02\r\n"); // enable only RMC and GGA
    UART_puts("$PCAS04,3*1A\r\n"); // GPS + BeiDou
    Delay_Ms(1000); // wait for disable to take action

    while(1) {
        poll_data();

        if (USART1->STATR & USART_STATR_RXNE) {
            // data in UART receive register
            char ch = USART1->DATAR;

            if (ch == '$') {
                // a new nmeamsg begins
                nmeamsg_len = 0;
                nmeamsg_checksum_calc = 0;
                nmeamsg_data_end = false;
            } else if (ch == '\n' || ch == '\r') {
                // nmeamsg ends. null terminate the message.
                nmeamsg[nmeamsg_len] = 0;

                if (nmeamsg_data_end && nmeamsg_len >= 2) {
                    // decode NMEA checksum from the end of the message
                    uint8_t nmeamsg_checksum_ref = hexpair_to_value(nmeamsg + nmeamsg_len - 2);

                    if (nmeamsg_checksum_ref == nmeamsg_checksum_calc) {
                        if ( (strncmp(nmeamsg, "GPRMC", 5) == 0) ||
                             (strncmp(nmeamsg, "GNRMC", 5) == 0)) {
                            // RMC message

                            int field_idx = 0;
                            int field_start = 0;

                            // time stamp broken down fields
                            int Y, M, D, h, m, s;
                            bool fix = false;

                            for(int i = 0; i < nmeamsg_len; i++) {
                                if (nmeamsg[i] == ',') {
                                    char* field_ptr = nmeamsg + field_start;
                                    int field_len = i - field_start;

                                    // an nmeamsg field ends. null terminate the field.
                                    nmeamsg[i] = 0;

                                    if (field_idx == 1) {
                                        // field 1 is UTC time as hhmmss
                                        if (field_len >= 6) {
                                            h = decpair_to_value(field_ptr + 0);
                                            m = decpair_to_value(field_ptr + 2);
                                            s = decpair_to_value(field_ptr + 4);
                                        }
                                    }
                                    if (field_idx == 2) {
                                        // field 2 is fix status
                                        if (field_len == 1) {
                                            if (field_ptr[0] == 'A') {
                                                fix = true;
                                            }
                                        }
                                    }
                                    if (field_idx == 9) {
                                        // field 9 is UTC date as DDMMYY
                                        if (field_len >= 6) {
                                            D = decpair_to_value(field_ptr + 0);
                                            M = decpair_to_value(field_ptr + 2);
                                            Y = decpair_to_value(field_ptr + 4) + 2000;
                                        }
                                    }

                                    field_idx++;
                                    field_start = i + 1;
                                }
                            }

                            update_timestamp(fix, Y, M, D, h, m, s);

                            if (i2c_master_done()) {
                                push_data_buf.timestamp = timestamp;
                                push_data_buf.fix = fix ? 1 : 0;
                                i2c_master_transfer(0x10, &push_data_buf, sizeof(push_data_buf));
                            }
                        } else if( (strncmp(nmeamsg, "GPGGA", 5) == 0) ||
                                   (strncmp(nmeamsg, "GNGGA", 5) == 0)) {
                            // GGA message

                            int field_idx = 0;
                            int field_start = 0;

                            // raw latitude and longitude data
                            int64_t lat, lon;
                            // number of satellites
                            uint8_t nsat;

                            for(int i = 0; i < nmeamsg_len; i++) {
                                if (nmeamsg[i] == ',') {
                                    char* field_ptr = nmeamsg + field_start;
                                    int field_len = i - field_start;

                                    // an nmeamsg field ends. null terminate the field.
                                    nmeamsg[i] = 0;

                                    if (field_idx == 2) {
                                        // field 2 is latitude as DDmm.mmmm...

                                        if (field_len >= 4) {
                                            // decode degrees and integer part of minutes
                                            int lat_deg = decpair_to_value(field_ptr + 0);
                                            int lat_min = decpair_to_value(field_ptr + 2);

                                            // decode fractional part of minutes
                                            int lat_min_frac = 0;
                                            int64_t scale = 1LL;
                                            for(int j = 5;j < field_len; j++) {
                                                lat_min_frac = lat_min_frac * 10 + digitvalue(*(field_ptr + j));
                                                scale *= 10LL;
                                            }

                                            // combine degrees, minutes and minute fractions at minute fraction resolution
                                            lat = (lat_deg * 60LL + lat_min) * scale + lat_min_frac;

                                            // scale to 360 / 2^32 degrees per LSB, i.e. approx 83.8 nanodegrees per LSB
                                            lat *= 0x8000000LL; // = 2^32 / 2^5
                                            lat /= 675LL;       // = 21600 / 2^5
                                            lat /= scale;
                                        }
                                    }

                                    if (field_idx == 3) {
                                        // field 3 is hemisphere as N or S
                                        if (field_len == 1) {
                                            if (*field_ptr == 'S') {
                                                lat *= -1;
                                            } else if (*field_ptr != 'N') {
                                                lat = 0;
                                            }
                                        }
                                    }

                                    if (field_idx == 4) {
                                        // field 4 is longitude as DDDmm.mmmm...
                                        if (field_len >= 5) {
                                            // decode degrees and integer part of minutes
                                            int lon_deg = dectriple_to_value(field_ptr + 0);
                                            int lon_min = decpair_to_value(field_ptr + 3);

                                            // decode fractional part of minutes
                                            int lon_min_frac = 0;
                                            int64_t scale = 1LL;
                                            for (int j = 6; j < field_len; j++) {
                                                lon_min_frac = lon_min_frac * 10 + digitvalue(*(field_ptr + j));
                                                scale *= 10LL;
                                            }

                                            // combine degrees, minutes and minute fractions at minute fraction resolution
                                            lon = (lon_deg * 60LL + lon_min) * scale + lon_min_frac;

                                            // scale to 360 / 2^32 degrees per LSB, i.e. approx 83.8 nanodegrees per LSB
                                            lon *= 0x8000000LL; // = 2^32 / 2^5
                                            lon /= 675LL;       // = 21600 / 2^5
                                            lon /= scale;
                                        }
                                    }

                                    if (field_idx == 5) {
                                        // field 5 is hemisphere as E or W
                                        if (field_len == 1) {
                                            if (*field_ptr == 'W') {
                                                lon *= -1;
                                            } else if(*field_ptr != 'E') {
                                                lon = 0;
                                            }
                                        }
                                    }

                                    if (field_idx == 7) {
                                        // field 7 is number of satellites
                                        if (field_len == 1) {
                                            nsat = digitvalue(*field_ptr);
                                        } else if (field_len == 2) {
                                            nsat = decpair_to_value(field_ptr);
                                        } else if (field_len == 3) {
                                            nsat = dectriple_to_value(field_ptr);
                                        } else {
                                            nsat = -1;
                                        }
                                    }

                                    field_idx++;
                                    field_start = i + 1;
                                }
                            }

                            update_auxiliary(lat, lon, nsat);
                        } else {
                            // received valid NMEA for unexpected message
                            // re-disable other messages
                            UART_puts("$PCAS03,1,0,0,0,1,0,0,0,,,,,,*02\r\n"); // enable only RMC and GGA
                        }
                    }
                }

                // clear message
                nmeamsg_len = 0;
                nmeamsg_checksum_calc = 0;
                nmeamsg_data_end = false;
            } else {
                // push received byte into nmeamsg if space
                if (nmeamsg_len < sizeof(nmeamsg)-1) {
                    nmeamsg[nmeamsg_len] = ch;
                    nmeamsg_len++;
                }

                // an asterisk delimits data from checksm
                if (ch == '*') {
                    nmeamsg_data_end = true;
                }

                // update checksum if byte was data
                if (nmeamsg_data_end == false) {
                    nmeamsg_checksum_calc ^= ch;
                }
            }
        }
    }
}
