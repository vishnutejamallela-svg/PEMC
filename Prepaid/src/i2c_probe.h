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

typedef struct
{
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t weekday;
    uint8_t hours;
    uint8_t minutes;
    uint8_t seconds;
} RTC_DateTime;

extern volatile RTC_DateTime rtc_datetime;
extern volatile uint8_t rtc_datetime_valid;

void I2C_Init(void);
void I2C_TestInit(void);
void I2C_RunReadOnlyTests(I2C_TestResult *result);
uint8_t RTC_ReadDateTime(volatile RTC_DateTime *datetime);
uint8_t RTC_WriteDateTime(const RTC_DateTime *datetime);
uint8_t RTC_SaveDateTimeToEEPROM(const RTC_DateTime *datetime);
uint8_t RTC_LoadDateTimeFromEEPROM(RTC_DateTime *datetime);
uint8_t I2C_EEPROM_Read(uint16_t address, uint8_t *buffer, uint8_t length);
uint8_t I2C_EEPROM_Write(uint16_t address, const uint8_t *buffer, uint8_t length);

#endif
