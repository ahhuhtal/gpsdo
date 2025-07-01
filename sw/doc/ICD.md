# Interface control documentation

Several messages pass between the components of the GPSDO system. The contents of those messages are described here.

## I2C

I2C messages are passed between the 4 microcontrollers which make up the system. Each controller has the ability to pull data from or push data to other controllers (act as a master), or have data pushed to or pulled from them (act as a slave).

Fields are described in the order they are transferred. All multi-byte fields are transferred as little-endian. All structures are packed without padding. 

### PPS message

- Type: master push
- 8-bit address: 0x10
- Master: `iface`
- Slave: `pfd`

Message structure:
```
uint32_t timestamp
uint8_t fix
```

`timestamp` is the unix timestamp of the received PPS pulse (seconds since 1970-01-01).

`fix` is 1 when the PPS was produced with full GNSS position fix. Any other value indicates that the PPS was produced without a valid fix.

### OCXO control message

- Type: master push
- 8-bit address: 0x0c
- Master: `pfd`
- Slave: `ocxo_ctrl`

Message structure:
```
int16_t control_word
```

`control_word` determines the frequency adjustment applied to the OCXO. The full scale is available for adjustment. An LSB corresponds with approximately 115 microHertz of frequency adjustment, or about 19 microvolts of control voltage.

### GNSS status message

- Type: master pull
- 8-bit address: 0x03
- Master: `lcd`
- Slave: `iface`

Message structure:
```
uint32_t timestamp
int32_t latitude
int32_t longitude
uint8_t nsat_fix_valid
```

`timestamp` is the unix timestamp of the last GNSS position fix (seconds since 1970-01-01).

`latitude` and `longitude` are the latitude and longitude of the last GNSS position fix as a fraction of 360 degrees, given in fixed point Q0.31 format (sign bit and 31 bits of fraction). Example: `degrees_latitude = latitude / 2**31 * 360.0`. Positive values of `latitude` indicate North, while positive values of `longitude` indicate East. 

Bit 7 of `nsat_fix_valid` indicates validity of the message. The message becomes invalid if the contents are older than 4 seconds.

Bit 6 of `nsat_fix_valid` indicates whether a GNSS position fix is valid (1) or not (0). A valid fix means that the latitude, longitude and timestamp values are valid.

Bits[5:0] of `nsat_fix_valid` indicate the number of satellites used in the last position fix. The value is interpreted as an unsigned integer in the range 0-63.

### Timing status message

- Type: master pull
- 8-bit address: 0x11
- Master: `lcd`
- Slave: `pfd`

Message structure:
```
int32_t phase_error_raw
int32_t phase_error_filtered
int32_t frequecy_error_raw
int32_t frequency_error_filtered
uint32_t time_since_valid
int16_t ocxo_control_word
uint8_t control_mode
```

`phase_error_raw` is the raw estimated phase error from the latest estimation period. `phase_error_filtered` is the filtered phase error, which is a low-pass filtered version of `phase_error_raw`. Both values are in fixed point Q7.24 format (sign bit, 7 bits of integer and 24 bits of fraction). The LSB of these values is approximately 3.0 fs (femtoseconds).

`frequency_error_raw` is the raw estimated frequency error from the latest estimation period. `frequency_error_filtered` is the filtered frequency error, which is a low-pass filtered version of `frequency_error_raw`. Both values are in fixed point Q7.24 format (sign bit, 7 bits of integer and 24 bits of fraction). The LSB of these values is approximately 29.8 nHz (nanohertz).

`time_since_valid` is the time in seconds since a PPS pulse passed full validation.

`ocxo_control_word` is the latest control word sent to the OCXO controller. It is in the same format as the `control_word` field of the OCXO control message.

`control_mode` indicates the current control mode of the system. A value of 0 indicates that the system is in FLL mode, while a value of 1 indicates that the system is in PLL mode. The system starts up in FLL mode and switches to PLL mode once the frequency error has been sufficiently reduced.

### OCXO status message

- Type: master pull
- 8-bit address: 0x0d
- Master: `lcd`
- Slave: `ocxo_ctrl`

Message structure:
```
int16_t temperature
```

`temperature` is the measured temperature of the OCXO. It is in Q11.4 format (sign bit, 11 bits of integer part, 4 bits of fraction) and is given in degrees Celsius. The LSB of this value is 0.0625 degrees Celsius. Example: `temperature_degrees = temperature / 16.0`.
