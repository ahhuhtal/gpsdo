#ifndef I2C_H
#define I2C_H

#include <stdint.h>

void i2c_init(uint8_t address, uint32_t bitrate);

void i2c_set_slave_tx(volatile void* buf, size_t len, void(*callback)(void));
void i2c_set_slave_rx(volatile void* buf, size_t len, void(*callback)(void));

void i2c_master_transfer(uint8_t address, volatile void* buf, size_t len);
bool i2c_master_done(void);
int i2c_master_error(void);

void i2c_acquire(void);
void i2c_release(void);

#endif
