#include "ch32v003fun.h"

#include <stdio.h>
#include <stdarg.h>
#include <stdbool.h>

#include <string.h>

#include "unixtime.h"

#define I2C1_CTLR1_DEFAULT (I2C_CTLR1_PE | I2C_CTLR1_ACK)
#define I2C1_CTLR2_DEFAULT (I2C_CTLR2_ITBUFEN | I2C_CTLR2_ITEVTEN | I2C_CTLR2_ITERREN)

volatile uint32_t time = 0;
volatile bool last_valid_timestamp = false;

volatile struct {
    uint32_t timestamp;
    uint8_t valid;
} txdata;

volatile uint8_t* txbuf = (uint8_t*)&txdata;

volatile uint32_t idx = 0;

// #define TRACING

void I2C1_EV_IRQHandler(void) __attribute__((interrupt));
void I2C1_EV_IRQHandler(void) {
    uint32_t s1 = I2C1->STAR1;
    uint32_t s2 = I2C1->STAR2;
    
#ifdef TRACING
    if (s2 & I2C_STAR2_MSL) {
        printf("We're a master.\n");
    } else {
        printf("We're a slave.\n");
    }

    if (s2 & I2C_STAR2_TRA) {
        printf("We're transmitting.\n");
    } else {
        printf("We're receiving\n");
    }
#endif

    if (s1 & I2C_STAR1_SB) {
#ifdef TRACING
        printf("Start bit sent\n");
#endif
        I2C1->DATAR = 0x10;
    }

    if (s1 & I2C_STAR1_ADDR) {
        idx = 0;
#ifdef TRACING
        printf("Address matched\n");
#endif
    }

    if (s1 & I2C_STAR1_RXNE) {
        (void)I2C1->DATAR;
#ifdef TRACING
        printf("Got byte\n");
#endif
    }

    if (s1 & I2C_STAR1_TXE) {
#ifdef TRACING
        printf("New byte requested\n");
#endif
        if (idx == 0) {
            txdata.timestamp = time;
            txdata.valid = last_valid_timestamp?1:0;
        }

        if (s2 & I2C_STAR2_MSL) {
            if (idx == 5) {
                I2C1->CTLR1 = I2C1_CTLR1_DEFAULT | I2C_CTLR1_STOP;
#ifdef TRACING
                printf("All sent. Requesting stop\n");
#endif
            } else {
                I2C1->DATAR = txbuf[idx];
                idx++;
            }
        } else {
            I2C1->DATAR = txbuf[idx];
            idx = (idx + 1) % 5;
        }
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
    uint32_t s2 = I2C1->STAR2;

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
        if (s2 & I2C_STAR2_MSL) {
#ifdef TRACING
            printf("Not acknowledge. Stopping\n");
#endif
            I2C1->CTLR1 = I2C1_CTLR1_DEFAULT | I2C_CTLR1_STOP;
            I2C1->STAR1 = s1 & (~I2C_STAR1_AF);
        } else {
#ifdef TRACING
            printf("Last byte. Stop.\n");
#endif
            I2C1->STAR1 = s1 & (~I2C_STAR1_AF);
        }
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

    // PC1, PC4: I2C pins
    GPIOC->CFGLR &= ~(0xf << (4*2));
    GPIOC->CFGLR |= (GPIO_Speed_2MHz | GPIO_CNF_OUT_OD_AF) << (4*2);
    GPIOC->CFGLR &= ~(0xf << (4*1));
    GPIOC->CFGLR |= (GPIO_Speed_2MHz | GPIO_CNF_OUT_OD_AF) << (4*1);

    NVIC_EnableIRQ(I2C1_EV_IRQn); // Event interrupt
    NVIC_SetPriority(I2C1_EV_IRQn, 0);
    NVIC_EnableIRQ(I2C1_ER_IRQn); // Error interrupt
    NVIC_SetPriority(I2C1_ER_IRQn, 0);

    I2C1->CKCFGR = FUNCONF_SYSTEM_CORE_CLOCK / (2*100000UL);
    I2C1->CTLR2 = FUNCONF_SYSTEM_CORE_CLOCK / 1000000UL;
    I2C1->OADDR1 = 0x0e;
    I2C1->CTLR1 = I2C1_CTLR1_DEFAULT;
    I2C1->CTLR2 = I2C1_CTLR2_DEFAULT;

    USART1->CTLR1 = USART_WordLength_8b | USART_Parity_No | USART_Mode_Rx;
    USART1->CTLR2 = USART_StopBits_1;
    USART1->CTLR3 = USART_HardwareFlowControl_None;
    USART1->BRR = FUNCONF_SYSTEM_CORE_CLOCK  / 115200UL;
    USART1->CTLR1 |= CTLR1_UE_Set;

    char nmeamsg[128];
    size_t nmeamsg_len = 0;

    uint8_t nmeamsg_checksum = 0;
    bool nmeamsg_data_end = false;

    // Config GNSS receiver for 115200 bps
    // Send the command at 9600 bps, which is default
    Delay_Ms(1000); // wait for receiver bootup
    UART_put("$PCAS01,5*19\r\n", 14);
    Delay_Ms(1000); // wait for baud rate change

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
                            NVIC_DisableIRQ(I2C1_EV_IRQn);
                            time = unix_from_utc(Y,M,D,h,m,s);
                            last_valid_timestamp = fix;
                            NVIC_EnableIRQ(I2C1_EV_IRQn);
                            I2C1->CTLR1 = I2C1_CTLR1_DEFAULT | I2C_CTLR1_START;
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
