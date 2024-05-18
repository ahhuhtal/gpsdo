#include "owi.h"

#include "ch32v003fun.h"

#define BSHR_SET_OFFSET 0
#define BSHR_RESET_OFFSET 16

#define OWI_setlow() { OWI_GPIO->BSHR = 1<<(BSHR_RESET_OFFSET+OWI_PIN); }
#define OWI_sethigh() { OWI_GPIO->BSHR = 1<<(BSHR_SET_OFFSET+OWI_PIN); }
#define OWI_sample() (((OWI_GPIO->INDR)&(1<<OWI_PIN)) != 0)

uint8_t _crc_ibutton_update(uint8_t crc, uint8_t data) {
	uint8_t i;
	crc = crc ^ data;
	for (i = 0; i < 8; i++) {
		if (crc & 0x01) {
            crc = (crc >> 1) ^ 0x8C;
		} else {
			crc >>= 1;
		}
    }
    return crc;
}

void OWI_init(void) {
    OWI_sethigh();
}

uint8_t OWI_reset(void) {
    uint8_t present;
    OWI_setlow();
    Delay_Us(480); // AN126
    OWI_sethigh();
    Delay_Us(70); // AN126
    if(OWI_sample()) {
        present=0;
    } else {
        present=1;
    }
    Delay_Us(410); // AN126
	return present;
}

void OWI_writebit(uint8_t bit) {
    if(bit) {
        OWI_setlow();
        Delay_Us(6); // AN126
        OWI_sethigh();
        Delay_Us(64); // AN126
    } else {
        OWI_setlow();
        Delay_Us(60); // AN126
        OWI_sethigh();
        Delay_Us(10); // AN126
    }
}

uint8_t OWI_readbit(void) {
    uint8_t bit;
    OWI_setlow();
    Delay_Us(6); // AN126
    OWI_sethigh();
    Delay_Us(9); // AN126
    if(OWI_sample()) {
        bit=1;
    } else {
        bit=0;
    }
    Delay_Us(55); // AN126

    return bit;
}

uint8_t OWI_readbyte(void) {
    uint8_t val=0;
    val|=(OWI_readbit()<<0);
    val|=(OWI_readbit()<<1);
    val|=(OWI_readbit()<<2);
    val|=(OWI_readbit()<<3);
    val|=(OWI_readbit()<<4);
    val|=(OWI_readbit()<<5);
    val|=(OWI_readbit()<<6);
    val|=(OWI_readbit()<<7);
    return val;
}

void OWI_writebyte(uint8_t byte) {
    OWI_writebit(byte&0x01);
    OWI_writebit(byte&0x02);
    OWI_writebit(byte&0x04);
    OWI_writebit(byte&0x08);
    OWI_writebit(byte&0x10);
    OWI_writebit(byte&0x20);
    OWI_writebit(byte&0x40);
    OWI_writebit(byte&0x80);
}

uint8_t OWI_checkcrc(uint8_t* data, uint8_t len) {
    uint8_t crc = 0;

    for(uint8_t i=0;i<len;i++) {
        crc = _crc_ibutton_update(crc, data[i]);
    }

    return crc; // must be 0
}

void OWI_matchrom(OWI_address address) {
    OWI_reset();

    uint8_t* address_bytes = (uint8_t*)&address;

    OWI_writebyte(0x55);

    for(uint8_t i=0;i<8;i++) {
        OWI_writebyte(address_bytes[i]);
    }
}

/* set family 0x28 resolution */
/* res: 0:  9 bits, 93.75ms conversion time
        1: 10 bits, 187.5ms
		2: 11 bits, 375 ms
		3: 12 bits, 750 ms */
void OWI_DS18B20setresolution(OWI_address address, uint8_t res) {
    OWI_matchrom(address);

    OWI_writebyte(0x4e);

    OWI_writebyte(0x00);
    OWI_writebyte(0x00);
    OWI_writebyte(res<<5);
}

/* request temperature measurement. request blocks until conversion complete */
void OWI_DS18B20convert(OWI_address address) {
    OWI_matchrom(address);

    OWI_writebyte(0x44);

    while(OWI_readbit()==0);
}

/* request temperature measurement. does not block */
void OWI_DS18B20startconvert(OWI_address address) {
    OWI_matchrom(address);

    OWI_writebyte(0x44);
}

/* is the previously selected device still busy? */
uint8_t OWI_busy(void) {
    return (OWI_readbit()==0);
}

/* read temperature value from device to argument value
   return value is the validity of CRC as 0: valid, nonzero: invalid */
uint8_t OWI_DS18B20readtemp(OWI_address address, int16_t* value) {
    OWI_matchrom(address);

    /* read scratchpad */
    OWI_writebyte(0xbe);

    union {
        uint8_t byte[2];
        int16_t word;
    } data;

    uint8_t crc=0;

    for(uint8_t i=0;i<9;i++) {
        uint8_t tmp=OWI_readbyte();
        if(i<=2) {
            data.byte[i]=tmp;
        }
        crc=_crc_ibutton_update(crc,tmp);
    }
    (*value)=data.word;
    return crc;
}

uint8_t OWI_searchrom(OWI_address* address, uint8_t lastDeviation) {
    uint8_t currentBit = 1;
    uint8_t newDeviation = 0;
    uint8_t bitMask = 0x01;
    uint8_t bitA;
    uint8_t bitB;

    uint8_t* bits=(uint8_t*)address;

    OWI_reset();

    // Send SEARCH ROM command on the bus.
    OWI_writebyte(0xf0);

    // Walk through all 64 bits.
    while (currentBit <= 64) {
        // Read bit from bus twice.
        bitA = OWI_readbit();
        bitB = OWI_readbit();

        if (bitA && bitB) {
            // Both bits 1 (Error).
            return 0xff;
        }
        else if (bitA ^ bitB) {
            // Bits A and B are different. All devices have the same bit here.
            // Set the bit in bitPattern to this value.
            if (bitA) {
                (*bits) |= bitMask;
            } else {
                (*bits) &= ~bitMask;
            }
        } else { // Both bits 0
            // If this is where a choice was made the last time,
            // a '1' bit is selected this time.
            if (currentBit == lastDeviation) {
                (*bits) |= bitMask;

            // For the rest of the id, '0' bits are selected when
            // discrepancies occur.
            } else if (currentBit > lastDeviation) {
                (*bits) &= ~bitMask;
                newDeviation = currentBit;

            // If current bit in bit pattern = 0, then this is
            // our new deviation.
            } else if ( !(*bits & bitMask))  {
                newDeviation = currentBit;

            // IF the bit is already 1, do nothing.
            } else {
            }
        }

        // Send the selected bit to the bus.
        if ((*bits) & bitMask) {
            OWI_writebit(1);
        } else {
            OWI_writebit(0);
        }

        // Increment current bit.
        currentBit++;

        // Adjust bitMask and bitPattern pointer.
        bitMask <<= 1;
        if (!bitMask) {
            bitMask = 0x01;
            bits++;
        }
    }
    return newDeviation;
}

uint8_t OWI_find_devices(OWI_address* device, uint8_t devcount_max) {
    /* check if there is even a single device connected */
    uint8_t presence=OWI_reset();
    if(!presence) {
        return 0;
    } else {
        /* there is at least one device */
        /* search for all connected 1-wire devices */
        uint8_t devices=0;
        uint8_t lastDeviation=0;

        do {
            OWI_reset();
            lastDeviation=OWI_searchrom(&(device[devices]),lastDeviation);
            devices++;
        } while(lastDeviation && devices<devcount_max);

        return devices;
    }
}
