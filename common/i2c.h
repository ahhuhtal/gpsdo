#ifndef I2C_H
#define I2C_H

#include <stdint.h>

/**
 * Low-effort I2C implementation.
 * Implements just enough features for this project.
 */

/**
 * Initialize the I2C peripheral
 * @param address Device 8 bit write address (7 bit address shifted left once)
 * @param bitrate Bitrate in bits per second
 */
void i2c_init(uint8_t address, uint32_t bitrate);

/**
 * Initialize slave transmitter mode
 * @param buf Pointer to buffer to hold tx data
 * @param len Number of tx data to send
 * @param callback Function pointer to call right before tx starts. Note: callback runs in an ISR context, so must not take long to execute.
 */
void i2c_set_slave_tx(volatile void* buf, size_t len, void(*callback)(void));

/**
 * Initialize slave receiver mode
 * @param buf Pointer to buffer to hold rx data
 * @param len Number of rx data to receive
 * @param callback Function pointer to call right after rx ends. Note: callback runs in an ISR context, so must not take long to execute.
 */
void i2c_set_slave_rx(volatile void* buf, size_t len, void(*callback)(void));

/**
 * Start transfer in master mode
 * @param address Address of slave. bit 0 clear = transmit to slave, bit 0 set = receive from slave.
 * @param buf Pointer to buffer to hold the rx or tx data
 * @param len Length of data to receive or transmit
 */
void i2c_master_transfer(uint8_t address, volatile void* buf, size_t len);

/**
 * Query state of the previous master transfer.
 * @return true if no transfer is ongoing, false is transfer is not complete
 */
bool i2c_master_done(void);

/**
 * Query error code from previous master transfer.
 * @return 0 = no error, 1 = slave did not acknowledge a byte, 2 = bus error
 */
int i2c_master_error(void);

/**
 * Acquire I2C lock.
 * Disables I2C interrupts. Prevents the I2C system from running asynchronously.
 * Note: the slave rx and tx callbacks run in ISR context, so locking not needed within.
 */
void i2c_acquire(void);

/**
 * Release I2C lock.
 * Enables I2C interrupts. Allows the I2C system to again run asynchronously.
 * Note: the slave rx and tx callbacks run in ISR context, so locking not needed within.
 */
void i2c_release(void);

#endif
