#ifndef OWI_H
#define OWI_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

// GPIO port and pin where 1-wire is connected
#define OWI_GPIO GPIOA
#define OWI_PIN 1

/**
 * Representation of 1-wire device on the bus
 */
typedef struct {
	uint8_t family; // device family
	uint8_t devaddr[6]; // device address
	uint8_t crc; // crc of the transaction
} __attribute__((__packed__)) OWI_address;

/**
 * Initialize the 1-wire system
 */
void OWI_init(void);

/**
 * Issue a reset on the 1-wire bus
 * @return false if no devices on bus, true if at least one device on bus
 */
bool OWI_reset();

/**
 * Find devices on the bus
 * @param address Pointer to an array of OWI_address structures to place found devices in
 * @param devices_max Length of device address structure array
 * @return Number of devices found on the bus
 */
size_t OWI_find_devices(OWI_address* address, size_t devices_max);

/**
 * Set temperature conversion resolution for a DS18B20 device
 * @param address The address of the DS18B20 device on the bus
 * @param res The temperature resolution to use:
 * 			  0:  9 bits, 93.75ms conversion time
 *            1: 10 bits, 187.5ms
 *            2: 11 bits, 375 ms
 *            3: 12 bits, 750 ms
 */
void OWI_DS18B20setresolution(OWI_address address, uint8_t res);

/**
 * Request a temperature conversion from DS18B20.
 * Blocks until conversion is complete.
 * @param address The address of the DS18B20 device on the bus
 */
void OWI_DS18B20convert(OWI_address address);

/**
 * Request a temperature conversion start from a DS18B20.
 * Does not wait for conversion completion
 * @param address The address of the DS18B20 device on the bus
 */
void OWI_DS18B20startconvert(OWI_address address);

/**
 * Is the previously selected device still busy?
 * @return true if busy, false if not
 */
bool OWI_busy(void);

/**
 * Read temperature conversion result from DS18B20
 * @param address Address of DS18B20 device on the bus
 * @param value Pointer to int16_t to store conversion result to
 * @return 0 on successful read, nonzero on CRC error
 */
uint8_t OWI_DS18B20readtemp(OWI_address address, int16_t* value);

/**
 * Check 1-wire CRC for a chunk of data
 * @param data Pointer to data buffer
 * @param len Length of data
 * @return 0 on CRC match, nonzero on CRC error
 */
uint8_t OWI_checkcrc(uint8_t* data, size_t len);

#endif
