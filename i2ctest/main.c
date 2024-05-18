#include <stdio.h>
#include <stdarg.h>
#include <stdbool.h>

#include "ch32v003fun.h"

#include "i2c.h"

uint32_t txdata = 0x12345678;
uint8_t rxdata[5];

void rx_done(void) {
    printf("rx done\n");
}

void tx_start(void) {
}

uint16_t buf;

int main() {
    SystemInit();

    i2c_init(0x10, 100000UL);
    i2c_set_slave_tx(&txdata, sizeof(txdata), tx_start);
    i2c_set_slave_rx(rxdata, sizeof(rxdata), rx_done);

    Delay_Ms(1000);

    while(1) {
        i2c_master_transfer(0x0c, &buf, sizeof(buf));
        while(!i2c_master_done());

        i2c_master_transfer(0x0d, &buf, sizeof(buf));
        while(!i2c_master_done());

        printf("done\n");
        Delay_Ms(100);
    }
}
