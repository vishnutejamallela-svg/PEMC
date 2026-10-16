#include "i2c_probe.h"

#define I2C_SI       0x08
#define I2C_STO      0x10
#define I2C_STA      0x20
#define I2C_ENABLE   0x40
#define EEPROM_ADDR  0x50
#define RTC_ADDR     0x51

static uint8_t I2C_WaitComplete(void)
{
    uint16_t timeout;

    timeout = 0;
    while ((I2CON & I2C_SI) == 0)
    {
        timeout++;
        if (timeout == 0) return 0;
    }
    return 1;
}

static uint8_t I2C_Start(void)
{
    SFRS = 0;
    I2CON |= I2C_STA;
    I2CON &= (uint8_t)~I2C_SI;
    if (!I2C_WaitComplete()) return 0;
    if (I2STAT != 0x08 && I2STAT != 0x10) return 0;
    I2CON &= (uint8_t)~I2C_STA;
    return 1;
}

static uint8_t I2C_Write(uint8_t value, uint8_t expected_status)
{
    I2DAT = value;
    I2CON &= (uint8_t)~I2C_SI;
    if (!I2C_WaitComplete()) return 0;
    return (I2STAT == expected_status);
}

static uint8_t I2C_Read(uint8_t acknowledge, uint8_t *value)
{
    if (acknowledge) I2CON |= 0x04;
    else I2CON &= (uint8_t)~0x04;

    I2CON &= (uint8_t)~I2C_SI;
    if (!I2C_WaitComplete()) return 0;
    if (acknowledge && I2STAT != 0x50) return 0;
    if (!acknowledge && I2STAT != 0x58) return 0;
    *value = I2DAT;
    return 1;
}

static uint8_t I2C_Stop(void)
{
    uint16_t timeout;

    SFRS = 0;
    I2CON |= I2C_STO;
    I2CON &= (uint8_t)~I2C_SI;
    timeout = 0;
    while (I2CON & I2C_STO)
    {
        timeout++;
        if (timeout == 0) return 0;
    }
    return 1;
}

static uint8_t I2C_Probe(uint8_t address)
{
    uint8_t acknowledged;

    acknowledged = 0;
    if (I2C_Start())
    {
        acknowledged = I2C_Write((uint8_t)(address << 1), 0x18);
    }
    return (uint8_t)(I2C_Stop() && acknowledged);
}

static uint8_t I2C_ReadRegister(uint8_t address, uint8_t register_address,
                                uint8_t *buffer, uint8_t length)
{
    uint8_t index;

    if (length == 0) return 0;
    if (!I2C_Start()) goto read_error;
    if (!I2C_Write((uint8_t)(address << 1), 0x18)) goto read_error;
    if (!I2C_Write(register_address, 0x28)) goto read_error;
    if (!I2C_Start()) goto read_error;
    if (!I2C_Write((uint8_t)((address << 1) | 1), 0x40)) goto read_error;

    for (index = 0; index < length; index++)
    {
        if (!I2C_Read(index + 1 < length, &buffer[index])) goto read_error;
    }
    return I2C_Stop();

read_error:
    I2C_Stop();
    return 0;
}

static uint8_t I2C_ReadEEPROMByte(uint8_t *value)
{
    if (!I2C_Start()) goto eeprom_error;
    if (!I2C_Write((uint8_t)(EEPROM_ADDR << 1), 0x18)) goto eeprom_error;
    if (!I2C_Write(0x00, 0x28)) goto eeprom_error;
    if (!I2C_Write(0x00, 0x28)) goto eeprom_error;
    if (!I2C_Start()) goto eeprom_error;
    if (!I2C_Write((uint8_t)((EEPROM_ADDR << 1) | 1), 0x40)) goto eeprom_error;
    if (!I2C_Read(0, value)) goto eeprom_error;
    return I2C_Stop();

eeprom_error:
    I2C_Stop();
    return 0;
}

void I2C_TestInit(void)
{
    SFRS = 0;
    P13_OPENDRAIN_MODE;
    P14_OPENDRAIN_MODE;
    P1 |= 0x18;
    I2CLK = 59;
    I2CON = I2C_ENABLE;
}

void I2C_RunReadOnlyTests(I2C_TestResult *result)
{
    uint8_t rtc_data[7];

    result->eeprom_present = I2C_Probe(EEPROM_ADDR);
    result->rtc_present = I2C_Probe(RTC_ADDR);
    result->eeprom_read_ok = 0;
    result->rtc_read_ok = 0;
    result->eeprom_first_byte = 0;
    result->rtc_seconds = 0;

    if (result->eeprom_present &&
        I2C_ReadEEPROMByte(&result->eeprom_first_byte))
    {
        result->eeprom_read_ok = 1;
    }

    if (result->rtc_present && I2C_ReadRegister(RTC_ADDR, 0x02, rtc_data, 7))
    {
        result->rtc_seconds = (uint8_t)(rtc_data[0] & 0x7F);
        result->rtc_read_ok = 1;
    }
}
