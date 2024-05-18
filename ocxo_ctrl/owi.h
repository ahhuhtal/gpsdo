#ifndef OWI_H
#define OWI_H

#include <stdint.h>

#define OWI_GPIO GPIOA
#define OWI_PIN 1

typedef struct {
	uint8_t family;
	uint8_t devaddr[6];
	uint8_t crc;
} __attribute__((__packed__)) OWI_address;

void OWI_init(void);

uint8_t OWI_reset();
uint8_t OWI_find_devices(OWI_address* address, uint8_t devcount_max);

void OWI_DS18B20setresolution(OWI_address address, uint8_t res);
void OWI_DS18B20convert(OWI_address address);
void OWI_DS18B20startconvert(OWI_address address);
uint8_t OWI_busy(void);

uint8_t OWI_DS18B20readtemp(OWI_address address, int16_t* value);

uint8_t OWI_checkcrc(uint8_t* data, uint8_t len);

#endif
