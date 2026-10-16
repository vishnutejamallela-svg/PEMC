#ifndef I2C_PROBE_H
#define I2C_PROBE_H

#include "ms51_device.h"

typedef struct
{
    uint8_t eeprom_present;
    uint8_t rtc_present;
    uint8_t eeprom_read_ok;
    uint8_t rtc_read_ok;
    uint8_t eeprom_first_byte;
    uint8_t rtc_seconds;
} I2C_TestResult;

void I2C_TestInit(void);
void I2C_RunReadOnlyTests(I2C_TestResult *result);

#endif
