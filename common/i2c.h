#ifndef I2C_H
#define I2C_H

#include <stdint.h>

void i2c_init(uint8_t address, uint32_t bitrate);

void i2c_set_slave_tx(void* buf, size_t len, void(*callback)(void));
void i2c_set_slave_rx(void* buf, size_t len, void(*callback)(void));

void i2c_master_transfer(uint8_t address, void* buf, size_t len);
bool i2c_master_done(void);

#endif
