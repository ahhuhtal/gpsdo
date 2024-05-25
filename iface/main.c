#include "ch32v003fun.h"

#include <stdio.h>
#include <stdarg.h>
#include <stdbool.h>

#include <string.h>

#include "unixtime.h"
#include "i2c.h"

// push and pull data

// timestamp data
volatile bool timestamp_valid;
volatile uint32_t timestamp_value;
volatile uint32_t timestamp_tick;

// auxiliary data
volatile int32_t latitude;
volatile int32_t longitude;
volatile uint8_t n_satellites;

void update_timestamp(bool valid, int Y, int M, int D, int h, int m, int s) {
    uint32_t value = unix_from_utc(Y,M,D,h,m,s);
    __disable_irq();
    timestamp_tick = SysTick->CNT;
    timestamp_valid = valid;
    timestamp_value = value;
    __enable_irq();
}

void poll_timestamp() {
    const uint32_t timeout = FUNCONF_SYSTEM_CORE_CLOCK/8;
    if (SysTick->CNT - timestamp_tick > timeout) {
        timestamp_valid = false;
    }
}

void update_auxiliary(int32_t lat, int32_t lon, uint8_t nsat) {
    __disable_irq();
    latitude = lat;
    longitude = lon;
    n_satellites = nsat;
    __enable_irq();
}

// packed struct when pulling data
volatile struct __attribute__((packed)) {
    uint32_t timestamp; // unix time of fix.
    int32_t lat; // fraction of 360 degrees in Q0.31 format. North is positive.
    int32_t lon; // fraction of 360 degrees in Q0.31 format. East is positive.
    uint8_t nsat_valid; // bit 7: validity, bits [6:0] number of satellites
} pull_data_buf;

// called when request to pull data is received
void update_tx_callback() {
    pull_data_buf.timestamp = timestamp_value;
    pull_data_buf.lat = latitude;
    pull_data_buf.lon = longitude;
    pull_data_buf.nsat_valid = n_satellites | (timestamp_valid?0x80:0x00);
}

void UART_bitbang(uint8_t ch) {
    uint16_t val = 0x200 | (ch << 1);
    for(int i=0;i<10;i++) {
        GPIOA->BSHR = 0x40000 | (val&0x01)<<2;
        DelaySysTick(FUNCONF_SYSTEM_CORE_CLOCK / (9600*8));
        val >>= 1;
    }
}

void UART_put(char* str, size_t len) {
    for(size_t i=0;i<len;i++) {
        UART_bitbang(str[i]);
    }
}

void UART_puts(char* str) {
    while(*str) {
        UART_bitbang(*str);
        str++;
    }
}

uint8_t digitvalue(char ch) {
    if (ch >= '0' && ch <='9') {
        return ch - '0';
    }
    if (ch>='A' && ch<='F') {
        return ch - 'A' + 0x0a;
    }
    if (ch>='a' && ch<='f') {
        return ch - 'a' + 0x0a;
    }
    return 0xff;
}

uint8_t hexpair_to_value(char* hex) {
    return (digitvalue(hex[0])<<4) | digitvalue(hex[1]);
}

uint8_t decpair_to_value(char* dec) {
    return digitvalue(dec[0])*10 + digitvalue(dec[1]);
}

uint8_t dectriple_to_value(char* dec) {
    return digitvalue(dec[0])*100 + digitvalue(dec[1])*10 + digitvalue(dec[2]);
}

int main() {
    SystemInit();

    RCC->APB2PCENR |= RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD | RCC_APB2Periph_USART1;
    RCC->APB1PCENR |= RCC_APB1Periph_I2C1;

    // PA2: SW bit-band UART TX
    GPIOA->CFGLR &= ~(0xf << (4*2));
    GPIOA->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_PP) << (4*2);
    GPIOA->BSHR = 1<<2;

    // PD6: USART1 RX
    GPIOC->CFGLR &= ~(0xf << (4*6));
    GPIOC->CFGLR |= (GPIO_Speed_In | GPIO_CNF_IN_FLOATING) << (4*6);

    i2c_init(0x03, 100000UL);
    i2c_set_slave_tx(&pull_data_buf, sizeof(pull_data_buf), update_tx_callback);

    USART1->CTLR1 = USART_WordLength_8b | USART_Parity_No | USART_Mode_Rx;
    USART1->CTLR2 = USART_StopBits_1;
    USART1->CTLR3 = USART_HardwareFlowControl_None;
    USART1->BRR = FUNCONF_SYSTEM_CORE_CLOCK  / 9600UL;
    USART1->CTLR1 |= CTLR1_UE_Set;

    char nmeamsg[128];
    size_t nmeamsg_len = 0;

    uint8_t nmeamsg_checksum = 0;
    bool nmeamsg_data_end = false;

    volatile struct __attribute__((packed)) {
        uint32_t timestamp;
        uint8_t valid;
    } push_data_buf;

    // UART_put("$PCAS01,5*19\r\n", 14); // set 115200

    // Config GNSS receiver to transmit less data
    Delay_Ms(1000); // wait for receiver bootup
    // UART_puts("$PCAS03,1,1,1,1,1,1,1,1,,,,,,*02\r\n"); // enable all messages
    // UART_puts("$PCAS03,0,0,0,0,1,0,0,0,,,,,,*03\r\n"); // enable only RMC
    UART_puts("$PCAS03,1,0,0,0,1,0,0,0,,,,,,*02\r\n"); // enable only RMC and GGA
    Delay_Ms(1000); // wait for disable to take action

    while(1) {
        if (USART1->STATR & USART_STATR_RXNE) {
            char ch = USART1->DATAR;

            if (ch=='$') {
                // a new nmeamsg begins
                nmeamsg_len = 0;
                nmeamsg_checksum = 0;
                nmeamsg_data_end = false;
            } else if (ch==10 || ch==13) {
                // nmeamsg ends. null terminate the message.
                nmeamsg[nmeamsg_len] = 0;

                if (nmeamsg_data_end && nmeamsg_len >= 2) {
                    uint8_t nmeamsg_checksum_data = hexpair_to_value(nmeamsg + nmeamsg_len - 2);

                    if (nmeamsg_checksum_data == nmeamsg_checksum) {
                        if (strncmp(nmeamsg, "GNRMC", 5) == 0) {
                            int field_idx = 0;
                            int field_start = 0;

                            int Y, M, D, h, m, s;
                            bool fix = false;

                            for(int i=0;i<nmeamsg_len;i++) {
                                if (nmeamsg[i] == ',') {
                                    char* field_ptr = nmeamsg + field_start;
                                    int field_len = i - field_start;

                                    // an nmeamsg field ends. null terminate the field.
                                    nmeamsg[i] = 0;

                                    if (field_idx==1) {
                                        if (field_len >= 6) {
                                            h = decpair_to_value(field_ptr+0);
                                            m = decpair_to_value(field_ptr+2);
                                            s = decpair_to_value(field_ptr+4);
                                        }
                                    }
                                    if (field_idx==2) {
                                        if (field_len == 1) {
                                            if (field_ptr[0] == 'A') {
                                                fix = true;
                                            }
                                        }
                                    }
                                    if (field_idx==9) {
                                        if (field_len >= 6) {
                                            D = decpair_to_value(field_ptr+0);
                                            M = decpair_to_value(field_ptr+2);
                                            Y = decpair_to_value(field_ptr+4) + 2000;
                                        }
                                    }
                                    field_idx++;
                                    field_start = i+1;
                                }
                            }

                            update_timestamp(fix, Y, M, D, h, m, s);
                            if (i2c_master_done()) {
                                push_data_buf.timestamp = timestamp_value;
                                push_data_buf.valid = timestamp_valid?1:0;
                                i2c_master_transfer(0x10, &push_data_buf, sizeof(push_data_buf));
                            }
                        } else if(strncmp(nmeamsg, "GNGGA", 5) == 0) {
                            int field_idx = 0;
                            int field_start = 0;

                            int64_t lat, lon;
                            uint8_t nsat;

                            for(int i=0;i<nmeamsg_len;i++) {
                                if (nmeamsg[i] == ',') {
                                    char* field_ptr = nmeamsg + field_start;
                                    int field_len = i - field_start;

                                    // an nmeamsg field ends. null terminate the field.
                                    nmeamsg[i] = 0;

                                    // latitude handling
                                    if (field_idx==2) {
                                        if (field_len >= 4) {
                                            int lat_deg = decpair_to_value(field_ptr+0);
                                            int lat_min = decpair_to_value(field_ptr+2);

                                            int lat_min_frac = 0;
                                            int64_t scale = 1LL;
                                            for (int j=5;j<field_len;j++) {
                                                lat_min_frac = lat_min_frac*10 + digitvalue(*(field_ptr+j));
                                                scale *= 10LL;
                                            }

                                            // maximal resolution as fractional minutes
                                            lat = (lat_deg*60LL + lat_min)*scale + lat_min_frac;

                                            // scale to 360/2^32 degrees per LSB.
                                            lat *= 0x8000000LL; // = 2^32 / 2^5
                                            lat /= 675LL;       // = 21600 / 2^5
                                            lat /= scale;
                                        }
                                    }
                                    if (field_idx==3) {
                                        if (field_len == 1) {
                                            if (*field_ptr == 'S') {
                                                lat *= -1;
                                            } else if(*field_ptr != 'N') {
                                                lat = 0;
                                            }
                                        }
                                    }

                                    // longitude handling
                                    if (field_idx==4) {
                                        if (field_len >= 5) {
                                            int lon_deg = dectriple_to_value(field_ptr+0);
                                            int lon_min = decpair_to_value(field_ptr+3);

                                            int lon_min_frac = 0;
                                            int64_t scale = 1LL;
                                            for (int j=6;j<field_len;j++) {
                                                lon_min_frac = lon_min_frac*10 + digitvalue(*(field_ptr+j));
                                                scale *= 10LL;
                                            }

                                            // maximal resolution as fractional minutes
                                            lon = (lon_deg*60LL + lon_min)*scale + lon_min_frac;

                                            // scale to 360/2^32 degrees per LSB.
                                            lon *= 0x8000000LL; // = 2^32 / 2^5
                                            lon /= 675LL;       // = 21600 / 2^5
                                            lon /= scale;
                                        }
                                    }
                                    if (field_idx==5) {
                                        if (field_len == 1) {
                                            if (*field_ptr == 'W') {
                                                lon *= -1;
                                            } else if(*field_ptr != 'E') {
                                                lon = 0;
                                            }
                                        }
                                    }

                                    if (field_idx==7) {
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
                                    field_start = i+1;
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
                nmeamsg_checksum = 0;
                nmeamsg_data_end = false;
            } else {
                // push received byte into nmeamsg if space
                if (nmeamsg_len < sizeof(nmeamsg)-1) {
                    nmeamsg[nmeamsg_len] = ch;
                    nmeamsg_len++;
                }

                // an asterisk delimits data from checksm
                if (ch=='*') {
                    nmeamsg_data_end = true;
                }

                // update checksum if byte was data
                if (nmeamsg_data_end == false) {
                    nmeamsg_checksum ^= ch;
                }
            }
        }
    }
}
