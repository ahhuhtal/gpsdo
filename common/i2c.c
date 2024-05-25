#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "ch32v003fun.h"

#include "i2c.h"

#define I2C_CTLR1_DEFAULT (I2C_CTLR1_PE | I2C_CTLR1_ACK)
#define I2C_CTLR1_NOACK I2C_CTLR1_PE

#define I2C_CTLR2_DEFAULT (I2C_CTLR2_ITBUFEN | I2C_CTLR2_ITEVTEN | I2C_CTLR2_ITERREN)

static volatile uint8_t* slave_tx_buf;
static volatile size_t slave_tx_buf_len;
static volatile bool slave_tx_busy;
void(*slave_tx_start_cb)(void);

static volatile uint8_t* slave_rx_buf;
static volatile size_t slave_rx_buf_len;
static volatile bool slave_rx_busy;
void(*slave_rx_done_cb)(void);

static volatile uint8_t master_address;
static volatile uint8_t* master_buf;
static volatile size_t master_buf_len;
static volatile bool master_busy;
static volatile bool master_error;

static volatile size_t transfer_index;

void I2C1_EV_IRQHandler(void) __attribute__((interrupt));
void I2C1_EV_IRQHandler(void) {
    uint16_t s1 = I2C1->STAR1;
    uint16_t s2 = I2C1->STAR2;
    
    bool master_mode = (s2 & I2C_STAR2_MSL) != 0;
    bool transmit_mode = (s2 & I2C_STAR2_TRA) != 0;

    if (s1 & I2C_STAR1_SB) {
        I2C1->DATAR = master_address;

        // if we're receiving only one byte,
        // the first received byte is the last byte.
        if (!transmit_mode && master_buf_len == 1) {
            I2C1->CTLR1 = I2C_CTLR1_NOACK;
        }
    }

    if (s1 & I2C_STAR1_ADDR) {
        // address was matched. transfer starts.
        transfer_index = 0;

        if (transmit_mode) {
            slave_tx_busy = true;
        } else {
            slave_rx_busy = true;
        }
    }

    if (s1 & I2C_STAR1_RXNE) {
        if (master_mode) {
            master_buf[transfer_index] = I2C1->DATAR;
            transfer_index++;

            if (transfer_index == master_buf_len - 1) {
                // next byte is the last byte.
                I2C1->CTLR1 = I2C_CTLR1_NOACK;
            } else if (transfer_index == master_buf_len) {
                I2C1->CTLR1 = I2C_CTLR1_DEFAULT | I2C_CTLR1_STOP;
                master_busy = false;
            }
        } else {
            slave_rx_buf[transfer_index] = I2C1->DATAR;
            transfer_index++;

            if (transfer_index == slave_rx_buf_len) {
                if (slave_rx_done_cb != NULL) {
                    slave_rx_done_cb();
                }
                transfer_index = 0;
            }
        }
    }

    if (s1 & I2C_STAR1_TXE) {
        if (master_mode) {
            if (transfer_index == master_buf_len) {
                I2C1->CTLR1 = I2C_CTLR1_DEFAULT | I2C_CTLR1_STOP;
                master_busy = false;
            } else {
                I2C1->DATAR = master_buf[transfer_index];
                transfer_index++;
            }
        } else {
            if (transfer_index == 0) {
                if (slave_tx_start_cb != NULL) {
                    slave_tx_start_cb();
                }
            }
            I2C1->DATAR = slave_tx_buf[transfer_index];
            transfer_index = (transfer_index + 1) % slave_tx_buf_len;
        }
    }

    if (s1 & I2C_STAR1_STOPF) {
        // write to CTLR1 to clear STOPF
        // read and write back to retain possible pending START flag
        I2C1->CTLR1 = I2C1->CTLR1;
        slave_rx_busy = false;
        slave_tx_busy = false;
    }
}

void I2C1_ER_IRQHandler(void) __attribute__((interrupt));
void I2C1_ER_IRQHandler(void) {
    uint16_t s1 = I2C1->STAR1;
    uint16_t s2 = I2C1->STAR2;

    bool master_mode = (s2 & I2C_STAR2_MSL) != 0;

    if (s1 & I2C_STAR1_PECERR) {
        // pec not enabled
        // should not happen
        I2C1->STAR1 = s1 & (~I2C_STAR1_PECERR);
    }
    if (s1 & I2C_STAR1_OVR) {
        // clock stretching should prevent this
        // should not happen
        I2C1->STAR1 = s1 & (~I2C_STAR1_OVR);
    }
    if (s1 & I2C_STAR1_AF) {
        if (master_mode) {
            // error. slave did not acknowledge.
            I2C1->STAR1 = s1 & (~I2C_STAR1_AF);
            I2C1->CTLR1 = I2C_CTLR1_DEFAULT | I2C_CTLR1_STOP;
            master_error = true;
            master_busy = false;
        } else {
            // normal. master does not acknowledge last received byte.
            I2C1->STAR1 = s1 & (~I2C_STAR1_AF);
            slave_tx_busy = false;
            slave_rx_busy = false;
        }
    }
    if (s1 & I2C_STAR1_ARLO) {
        I2C1->STAR1 = s1 & (~I2C_STAR1_ARLO);
        // we lost arbitration. try again.
        I2C1->CTLR1 = I2C_CTLR1_DEFAULT | I2C_CTLR1_START;
    }
    if (s1 & I2C_STAR1_BERR) {
        I2C1->STAR1 = s1 & (~I2C_STAR1_BERR);
        master_busy = false;
        master_error = true;
        slave_tx_busy = false;
        slave_rx_busy = false;
    }
}

void i2c_init(uint8_t address, uint32_t bitrate) {
    RCC->APB2PCENR |= RCC_APB2Periph_GPIOC;
    RCC->APB1PCENR |= RCC_APB1Periph_I2C1;

    // PC1, PC4: I2C pins
    GPIOC->CFGLR &= ~(0xf << (4*2));
    GPIOC->CFGLR |= (GPIO_Speed_2MHz | GPIO_CNF_OUT_OD_AF) << (4*2);
    GPIOC->CFGLR &= ~(0xf << (4*1));
    GPIOC->CFGLR |= (GPIO_Speed_2MHz | GPIO_CNF_OUT_OD_AF) << (4*1);

    NVIC_SetPriority(I2C1_EV_IRQn, 0);
    NVIC_SetPriority(I2C1_ER_IRQn, 0);

    NVIC_EnableIRQ(I2C1_EV_IRQn);
    NVIC_EnableIRQ(I2C1_ER_IRQn);

    I2C1->CKCFGR = FUNCONF_SYSTEM_CORE_CLOCK / (2*bitrate);
    I2C1->CTLR2 = FUNCONF_SYSTEM_CORE_CLOCK / 1000000UL;
    I2C1->OADDR1 = address;
    I2C1->CTLR1 = I2C_CTLR1_DEFAULT;
    I2C1->CTLR2 = I2C_CTLR2_DEFAULT;
}

static void acquire(void) {
    NVIC_DisableIRQ(I2C1_EV_IRQn);
    NVIC_DisableIRQ(I2C1_ER_IRQn);
}

static void release(void) {
    NVIC_EnableIRQ(I2C1_EV_IRQn);
    NVIC_EnableIRQ(I2C1_ER_IRQn);
}

void i2c_set_slave_tx(volatile void* buf, size_t len, void(*callback)(void)) {
    acquire();
    while (slave_tx_busy) {
        release();
        acquire();
    }
    slave_tx_buf = buf;
    slave_tx_buf_len = len;
    slave_tx_start_cb = callback;
    release();
}

void i2c_set_slave_rx(volatile void* buf, size_t len, void(*callback)(void)) {
    acquire();
    while (slave_tx_busy) {
        release();
        acquire();
    }
    slave_rx_buf = buf;
    slave_rx_buf_len = len;
    slave_rx_done_cb = callback;
    release();
}

void i2c_master_transfer(uint8_t address, volatile void* buf, size_t len) {
    acquire();
    while (master_busy) {
        release();
        acquire();
    }
    master_address = address;
    master_buf = buf;
    master_buf_len = len;
    master_error = false;
    master_busy = true;
    I2C1->CTLR1 = I2C_CTLR1_DEFAULT | I2C_CTLR1_START;
    release();
}

bool i2c_master_done(void) {
    return !master_busy;
}

bool i2c_master_error(void) {
    return master_error;
}
