#include "i2c_probe.h"
#include "board_io.h"

#define I2C_SI       0x08
#define I2C_STO      0x10
#define I2C_STA      0x20
#define I2C_ENABLE   0x40
#define EEPROM_ADDR  0x50
#define RTC_ADDR     0x51
#define RTC_EEPROM_OFFSET 0x0040
#define RTC_EEPROM_RECORD_SIZE 13

volatile RTC_DateTime rtc_datetime;
volatile uint8_t rtc_datetime_valid;

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

static uint8_t I2C_WriteRegister(uint8_t address, uint8_t register_address,
                                 const uint8_t *buffer, uint8_t length)
{
    uint8_t index;

    if (length == 0) return 0;
    if (!I2C_Start()) goto write_error;
    if (!I2C_Write((uint8_t)(address << 1), 0x18)) goto write_error;
    if (!I2C_Write(register_address, 0x28)) goto write_error;
    for (index = 0; index < length; index++)
    {
        if (!I2C_Write(buffer[index], 0x28)) goto write_error;
    }
    return I2C_Stop();

write_error:
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

static uint8_t I2C_ReadEEPROM(uint16_t address, uint8_t *buffer,
                              uint8_t length)
{
    uint8_t index;

    if (length == 0 || (uint32_t)address + length > 8192UL) return 0;
    if (!I2C_Start()) goto eeprom_read_error;
    if (!I2C_Write((uint8_t)(EEPROM_ADDR << 1), 0x18)) goto eeprom_read_error;
    if (!I2C_Write((uint8_t)(address >> 8), 0x28)) goto eeprom_read_error;
    if (!I2C_Write((uint8_t)address, 0x28)) goto eeprom_read_error;
    if (!I2C_Start()) goto eeprom_read_error;
    if (!I2C_Write((uint8_t)((EEPROM_ADDR << 1) | 1), 0x40))
        goto eeprom_read_error;
    for (index = 0; index < length; index++)
    {
        if (!I2C_Read(index + 1 < length, &buffer[index]))
            goto eeprom_read_error;
    }
    return I2C_Stop();

eeprom_read_error:
    I2C_Stop();
    return 0;
}

static uint8_t I2C_WriteEEPROM(uint16_t address, const uint8_t *buffer,
                               uint8_t length)
{
    uint8_t index;

    if (length == 0 || (uint32_t)address + length > 8192UL ||
        (address / 32) != ((address + length - 1) / 32))
    {
        return 0;
    }
    if (!I2C_Start()) goto eeprom_write_error;
    if (!I2C_Write((uint8_t)(EEPROM_ADDR << 1), 0x18))
        goto eeprom_write_error;
    if (!I2C_Write((uint8_t)(address >> 8), 0x28))
        goto eeprom_write_error;
    if (!I2C_Write((uint8_t)address, 0x28))
        goto eeprom_write_error;
    for (index = 0; index < length; index++)
    {
        if (!I2C_Write(buffer[index], 0x28)) goto eeprom_write_error;
    }
    if (!I2C_Stop()) return 0;
    Board_DelayMs(5);
    return 1;

eeprom_write_error:
    I2C_Stop();
    return 0;
}

uint8_t I2C_EEPROM_Read(uint16_t address, uint8_t *buffer, uint8_t length)
{
    return I2C_ReadEEPROM(address, buffer, length);
}

uint8_t I2C_EEPROM_Write(uint16_t address, const uint8_t *buffer, uint8_t length)
{
    uint8_t written;
    uint8_t page_remaining;
    uint8_t chunk;

    if (buffer == 0 || length == 0 || (uint32_t)address + length > 8192UL)
        return 0;
    written = 0;
    while (written < length)
    {
        page_remaining = (uint8_t)(32 - ((address + written) % 32));
        chunk = (uint8_t)(length - written);
        if (chunk > page_remaining) chunk = page_remaining;
        if (!I2C_WriteEEPROM(address + written, buffer + written, chunk))
            return 0;
        written += chunk;
    }
    return 1;
}

static uint16_t RTC_EEPROM_Crc16(const uint8_t *buffer, uint8_t length)
{
    uint16_t crc;
    uint8_t index;
    uint8_t bit_index;

    crc = 0xFFFF;
    for (index = 0; index < length; index++)
    {
        crc ^= buffer[index];
        for (bit_index = 0; bit_index < 8; bit_index++)
        {
            if (crc & 1)
            {
                crc = (uint16_t)((crc >> 1) ^ 0xA001);
            }
            else
            {
                crc >>= 1;
            }
        }
    }
    return crc;
}

static uint8_t I2C_BcdToBinary(uint8_t value, uint8_t mask, uint8_t maximum,
                               uint8_t *result)
{
    uint8_t tens;
    uint8_t units;
    uint8_t decoded;

    value &= mask;
    tens = (uint8_t)((value >> 4) & 0x0F);
    units = (uint8_t)(value & 0x0F);
    if (tens > 9 || units > 9)
    {
        return 0;
    }
    decoded = (uint8_t)((tens * 10) + units);
    if (decoded > maximum)
    {
        return 0;
    }
    *result = decoded;
    return 1;
}

static uint8_t RTC_BinaryToBcd(uint8_t value)
{
    return (uint8_t)(((value / 10) << 4) | (value % 10));
}

static uint8_t RTC_IsLeapYear(uint16_t year)
{
    return (uint8_t)(((year % 4) == 0) &&
        (((year % 100) != 0) || ((year % 400) == 0)));
}

static uint8_t RTC_DaysInMonth(uint16_t year, uint8_t month)
{
    switch (month)
    {
        case 2:
            return RTC_IsLeapYear(year) ? 29 : 28;
        case 4:
        case 6:
        case 9:
        case 11:
            return 30;
        default:
            return 31;
    }
}

static uint8_t RTC_DateTimeIsValid(const RTC_DateTime *datetime)
{
    if (datetime == 0 || datetime->year < 2000 || datetime->year > 2199 ||
        datetime->month < 1 || datetime->month > 12 || datetime->day < 1 ||
        datetime->day > RTC_DaysInMonth(datetime->year, datetime->month) ||
        datetime->weekday > 6 || datetime->hours > 23 ||
        datetime->minutes > 59 || datetime->seconds > 59)
    {
        return 0;
    }
    return 1;
}

uint8_t RTC_SaveDateTimeToEEPROM(const RTC_DateTime *datetime)
{
    uint8_t record[RTC_EEPROM_RECORD_SIZE];
    uint16_t crc;
    uint8_t verify[RTC_EEPROM_RECORD_SIZE];
    uint8_t index;

    if (!RTC_DateTimeIsValid(datetime)) return 0;

    record[0] = 0x52;
    record[1] = 0x54;
    record[2] = 0x01;
    record[3] = (uint8_t)(datetime->year >> 8);
    record[4] = (uint8_t)datetime->year;
    record[5] = datetime->month;
    record[6] = datetime->day;
    record[7] = datetime->weekday;
    record[8] = datetime->hours;
    record[9] = datetime->minutes;
    record[10] = datetime->seconds;
    crc = RTC_EEPROM_Crc16(record, 11);
    record[11] = (uint8_t)crc;
    record[12] = (uint8_t)(crc >> 8);

    if (!I2C_WriteEEPROM(RTC_EEPROM_OFFSET, record,
                         RTC_EEPROM_RECORD_SIZE) ||
        !I2C_ReadEEPROM(RTC_EEPROM_OFFSET, verify,
                        RTC_EEPROM_RECORD_SIZE))
    {
        return 0;
    }
    for (index = 0; index < RTC_EEPROM_RECORD_SIZE; index++)
    {
        if (record[index] != verify[index]) return 0;
    }
    return 1;
}

uint8_t RTC_LoadDateTimeFromEEPROM(RTC_DateTime *datetime)
{
    uint8_t record[RTC_EEPROM_RECORD_SIZE];
    uint16_t stored_crc;
    uint16_t calculated_crc;
    RTC_DateTime candidate;

    if (datetime == 0 ||
        !I2C_ReadEEPROM(RTC_EEPROM_OFFSET, record,
                        RTC_EEPROM_RECORD_SIZE))
    {
        return 0;
    }
    if (record[0] != 0x52 || record[1] != 0x54 || record[2] != 0x01)
    {
        return 0;
    }

    stored_crc = (uint16_t)(record[11] | ((uint16_t)record[12] << 8));
    calculated_crc = RTC_EEPROM_Crc16(record, 11);
    if (stored_crc != calculated_crc) return 0;

    candidate.year = (uint16_t)(((uint16_t)record[3] << 8) | record[4]);
    candidate.month = record[5];
    candidate.day = record[6];
    candidate.weekday = record[7];
    candidate.hours = record[8];
    candidate.minutes = record[9];
    candidate.seconds = record[10];
    if (!RTC_DateTimeIsValid(&candidate)) return 0;

    *datetime = candidate;
    return 1;
}

uint8_t RTC_WriteDateTime(const RTC_DateTime *datetime)
{
    uint8_t rtc_data[7];

    if (!RTC_DateTimeIsValid(datetime)) return 0;

    rtc_data[0] = RTC_BinaryToBcd(datetime->seconds);
    rtc_data[1] = RTC_BinaryToBcd(datetime->minutes);
    rtc_data[2] = RTC_BinaryToBcd(datetime->hours);
    rtc_data[3] = RTC_BinaryToBcd(datetime->day);
    rtc_data[4] = (uint8_t)(datetime->weekday & 0x07);
    rtc_data[5] = RTC_BinaryToBcd(datetime->month);
    if (datetime->year >= 2100)
    {
        rtc_data[5] |= 0x80;
    }
    rtc_data[6] = RTC_BinaryToBcd((uint8_t)(datetime->year % 100));

    if (!I2C_WriteRegister(RTC_ADDR, 0x02, rtc_data, 7))
    {
        return 0;
    }
    rtc_datetime_valid = 1;
    return 1;
}

uint8_t RTC_ReadDateTime(volatile RTC_DateTime *datetime)
{
    uint8_t rtc_data[7];
    uint8_t year;
    uint8_t month;
    uint8_t day;
    uint8_t weekday;
    uint8_t hours;
    uint8_t minutes;
    uint8_t seconds;

    rtc_datetime_valid = 0;
    if (datetime == 0 || !I2C_ReadRegister(RTC_ADDR, 0x02, rtc_data, 7))
    {
        return 0;
    }
    if ((rtc_data[0] & 0x80) != 0 ||
        !I2C_BcdToBinary(rtc_data[0], 0x7F, 59, &seconds) ||
        !I2C_BcdToBinary(rtc_data[1], 0x7F, 59, &minutes) ||
        !I2C_BcdToBinary(rtc_data[2], 0x3F, 23, &hours) ||
        !I2C_BcdToBinary(rtc_data[3], 0x3F, 31, &day) ||
        !I2C_BcdToBinary(rtc_data[5], 0x1F, 12, &month) ||
        !I2C_BcdToBinary(rtc_data[6], 0xFF, 99, &year))
    {
        return 0;
    }
    weekday = (uint8_t)(rtc_data[4] & 0x07);
    if (day == 0 || month == 0 || weekday > 6)
    {
        return 0;
    }

    datetime->year = (uint16_t)(2000 + year +
        ((rtc_data[5] & 0x80) ? 100 : 0));
    datetime->month = month;
    datetime->day = day;
    datetime->weekday = weekday;
    datetime->hours = hours;
    datetime->minutes = minutes;
    datetime->seconds = seconds;
    rtc_datetime_valid = 1;
    return 1;
}

void I2C_Init(void)
{
    SFRS = 0;
    P13_OPENDRAIN_MODE;
    P14_OPENDRAIN_MODE;
    P1 |= 0x18;
    I2CLK = 59;
    I2CON = I2C_ENABLE;
}

void I2C_TestInit(void)
{
    I2C_Init();
}

void I2C_RunReadOnlyTests(I2C_TestResult *result)
{
    RTC_DateTime rtc_time;

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

    if (result->rtc_present && RTC_ReadDateTime(&rtc_time))
    {
        result->rtc_seconds = rtc_time.seconds;
        result->rtc_read_ok = 1;
    }
}
