#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "ch32fun.h"

#include "i2c.h"

// default resting state of CTLR1. slave mode always has this state.
#define I2C_CTLR1_DEFAULT (I2C_CTLR1_PE | I2C_CTLR1_ACK)

static volatile uint8_t* slave_tx_buf; // pointer to slave tx data
static volatile size_t slave_tx_buf_len; // length of slave tx data
static volatile bool slave_tx_busy; // true if slave tx is in progress
void(*slave_tx_start_cb)(void); // function pointer to slave tx callback, called right before data transfer starts

static volatile uint8_t* slave_rx_buf; // pointer to slave rx data buffer
static volatile size_t slave_rx_buf_len; // length of slave rx data buffer
static volatile bool slave_rx_busy; // true if slave rx is in progres
void(*slave_rx_done_cb)(void); // function pointer to slave rx callback, called after rx buffer is filled

static volatile uint8_t master_slave_address; // 8 bit address of slave to talk to in master mode
static volatile uint8_t* master_buf; // pointer to master tx/rx data buffer
static volatile size_t master_buf_len; // length of master rx or tx data
static volatile bool master_busy; // true if master tx or rx is busy
static volatile int master_error; // last error condition of a master transfer

static volatile size_t transfer_index; // data transfer byte counter

/**
 * I2C event interrupt.
 * Executed for state changes. Handles both master and slave transfer logic.
 */
void I2C1_EV_IRQHandler(void) __attribute__((interrupt));
void I2C1_EV_IRQHandler(void) {
    uint16_t s1 = I2C1->STAR1;
    uint16_t s2 = I2C1->STAR2;

    // check which roles we are in
    bool master_mode = (s2 & I2C_STAR2_MSL) != 0; // master = true, slave = false
    bool transmit_mode = (s2 & I2C_STAR2_TRA) != 0; // transmitter = true, receiver = false

    if (s1 & I2C_STAR1_SB) {
        // we succesfully issued a start bit.

        // transmit the address of the slave we're talking with.
        I2C1->DATAR = master_slave_address;

        // if we're receiving only one byte,
        // the first received byte is the last byte.
        if (!transmit_mode && master_buf_len == 1) {
            // receive the byte, but do not acknowledge it
            I2C1->CTLR1 = I2C_CTLR1_PE;
        }
    }

    if (s1 & I2C_STAR1_ADDR) {
        // master mode: slave acknowledged its address
        // slave mode: we acknowledged our address

        // data transfer starts.
        transfer_index = 0;

        if (!master_mode) {
            if (transmit_mode) {
                slave_tx_busy = true;

                // call the tx callback to allow preparation of data
                if (slave_tx_start_cb != NULL) {
                    slave_tx_start_cb();
                }
            } else {
                slave_rx_busy = true;
            }
        }
    }

    if (s1 & I2C_STAR1_RXNE) {
        // data was received into the receive register

        if (master_mode) {
            master_buf[transfer_index] = I2C1->DATAR;
            transfer_index++;

            if (transfer_index == master_buf_len) {
                // this was the last byte. issue a stop and return to default state
                I2C1->CTLR1 = I2C_CTLR1_DEFAULT | I2C_CTLR1_STOP;
                master_busy = false;
            } else if (transfer_index == master_buf_len - 1) {
                // next byte is the last byte. receive it, but do not acknowledge it.
                I2C1->CTLR1 = I2C_CTLR1_PE;
            } // else the next byte is not the last, continue as usual by receiving and acknowledgeing
        } else {
            slave_rx_buf[transfer_index] = I2C1->DATAR;
            transfer_index++;

            if (transfer_index == slave_rx_buf_len) {
                // this was the last byte. call the rx callback.
                if (slave_rx_done_cb != NULL) {
                    slave_rx_done_cb();
                }

                // if master continues pushing more data, just wrap the buffer.
                transfer_index = 0;
            }
        }
    }

    if (s1 & I2C_STAR1_TXE) {
        // data is expected in the transmit register

        if (master_mode) {
            if (transfer_index == master_buf_len) {
                // previous byte sent was the last byte. issue a stop and return to default state.
                I2C1->CTLR1 = I2C_CTLR1_DEFAULT | I2C_CTLR1_STOP;
                master_busy = false;
            } else {
                I2C1->DATAR = master_buf[transfer_index];
                transfer_index++;
            }
        } else {
            // transmit data in the tx buffer, wrapping it around if more is requested.
            I2C1->DATAR = slave_tx_buf[transfer_index];
            transfer_index = (transfer_index + 1) % slave_tx_buf_len;
        }
    }

    if (s1 & I2C_STAR1_STOPF) {
        // stop bit was detected.

        // write to CTLR1 to clear STOPF
        // read and write back to retain possible pending START flag
        I2C1->CTLR1 = I2C1->CTLR1;
        slave_rx_busy = false;
        slave_tx_busy = false;
    }
}

/**
 * I2C error interrupt.
 * Executed for error conditions. Handles error signaling and state restore.
 */
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
            // error. slave did not acknowledge. issue a stop and return to default state.
            I2C1->STAR1 = s1 & (~I2C_STAR1_AF);
            I2C1->CTLR1 = I2C_CTLR1_DEFAULT | I2C_CTLR1_STOP;
            master_error = 1;
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
        // we lost arbitration. try issuing a start bit again.
        I2C1->CTLR1 = I2C_CTLR1_DEFAULT | I2C_CTLR1_START;
    }
    if (s1 & I2C_STAR1_BERR) {
        I2C1->STAR1 = s1 & (~I2C_STAR1_BERR);
        master_busy = false;
        master_error = 2;
        slave_tx_busy = false;
        slave_rx_busy = false;
    }
}

void i2c_init(uint8_t address, uint32_t bitrate) {
    RCC->APB2PCENR |= RCC_APB2Periph_GPIOC;
    RCC->APB1PCENR |= RCC_APB1Periph_I2C1;

    // PC1, PC4: I2C pins
    GPIOC->CFGLR &= ~(0xf << (4 * 2));
    GPIOC->CFGLR |= (GPIO_Speed_2MHz | GPIO_CNF_OUT_OD_AF) << (4 * 2);
    GPIOC->CFGLR &= ~(0xf << (4 * 1));
    GPIOC->CFGLR |= (GPIO_Speed_2MHz | GPIO_CNF_OUT_OD_AF) << (4 * 1);

    NVIC_EnableIRQ(I2C1_EV_IRQn);
    NVIC_EnableIRQ(I2C1_ER_IRQn);

    // configure own address
    I2C1->OADDR1 = address;

    // configure bitrate with fast-mode = 0, duty = 0
    // in this mode, ccr = fcpu / (2 * bitrate)
    uint16_t ccr = (FUNCONF_SYSTEM_CORE_CLOCK / (2 * bitrate)) & 0xfff;
    I2C1->CKCFGR = ccr;

    // configure peripheral setup and hold with freq field
    // freq = integer number of megahertz at which the i2c core is clocked at
    uint8_t freq = (FUNCONF_SYSTEM_CORE_CLOCK / 1000000UL) & 0x3f;
    I2C1->CTLR2 = I2C_CTLR2_ITBUFEN | I2C_CTLR2_ITEVTEN | I2C_CTLR2_ITERREN | freq;

    // enable peripheral
    I2C1->CTLR1 = I2C_CTLR1_DEFAULT;
}

void i2c_acquire(void) {
    NVIC_DisableIRQ(I2C1_EV_IRQn);
    NVIC_DisableIRQ(I2C1_ER_IRQn);
}

void i2c_release(void) {
    NVIC_EnableIRQ(I2C1_EV_IRQn);
    NVIC_EnableIRQ(I2C1_ER_IRQn);
}

void i2c_set_slave_tx(volatile void* buf, size_t len, void(*callback)(void)) {
    // lock i2c to allow configuration atomically
    i2c_acquire();

    // if we were in the middle of a transfer, we must wait it to end
    while (slave_tx_busy) {
        i2c_release();
        // release lock here to allow I2C state progressing
        i2c_acquire();
    }

    slave_tx_buf = buf;
    slave_tx_buf_len = len;
    slave_tx_start_cb = callback;
    i2c_release();
}

void i2c_set_slave_rx(volatile void* buf, size_t len, void(*callback)(void)) {
    // lock i2c to allow configuration atomically
    i2c_acquire();

    // if we were in the middle of a transfer, we must wait it to end
    while (slave_rx_busy) {
        i2c_release();
        // release lock here to allow I2C state progressing
        i2c_acquire();
    }

    slave_rx_buf = buf;
    slave_rx_buf_len = len;
    slave_rx_done_cb = callback;
    i2c_release();
}

void i2c_master_transfer(uint8_t address, volatile void* buf, size_t len) {
    // if we were in the middle of a transfer, we must wait it to end
    // new master transfers cannot start, so no need to acquire lock
    while (master_busy);

    master_slave_address = address;
    master_buf = buf;
    master_buf_len = len;
    master_error = 0;
    master_busy = true;

    // issue a start bit as soon as possible.
    // this means waiting for slave mode communication to end
    // and/or the other communication on the bus to end.
    I2C1->CTLR1 = I2C_CTLR1_DEFAULT | I2C_CTLR1_START;
}

bool i2c_master_done(void) {
    return !master_busy;
}

int i2c_master_error(void) {
    return master_error;
}
