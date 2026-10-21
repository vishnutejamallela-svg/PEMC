#include "rs485_test.h"
#include "board_io.h"
#include "i2c_probe.h"
#include "lcd_595.h"
#include "uart.h"

#define METER_SLAVE_ID          4
#define CONTROLLER_SLAVE_ID     1
#define METER_BAUD_RATE         9600UL
/* Raw Modbus offset for the ML14XX 4X block beginning at 40155. */
#define METER_REGISTER_START    154
#define METER_REGISTER_COUNT    ENERGY_METER_REGISTER_COUNT
#define SLAVE_REGISTER_COUNT    SLAVE_HOLDING_REGISTER_COUNT
#define PEMC_CONTROLLER_ID_OFFSET       0U
#define PEMC_PARITY_OFFSET              1U
#define PEMC_BAUD_START                 2U
#define PEMC_BACKLIGHT_TIMEOUT_OFFSET   4U
#define PEMC_RTC_YEAR_OFFSET            5U
#define PEMC_RTC_MONTH_OFFSET           6U
#define PEMC_RTC_DAY_OFFSET             7U
#define PEMC_RTC_HOUR_OFFSET            8U
#define PEMC_RTC_MINUTE_OFFSET          9U
#define PEMC_RTC_SECOND_OFFSET         10U
#define PEMC_MAINS_RATE_START          11U
#define PEMC_DG_RATE_START             13U
#define PEMC_SOLAR_RATE_START          15U
#define PEMC_LOW_BALANCE_START         17U
#define PEMC_RELAY_COMMAND_OFFSET      19U
#define PEMC_BALANCE_SOURCE_OFFSET     20U
#define PEMC_CUSTOMER_ID_START         21U
#define EM_SETTINGS_SLAVE_ID_OFFSET    50U
#define EM_SETTINGS_PARITY_OFFSET      51U
#define EM_SETTINGS_BAUD_START         52U
#define EM_SETTINGS_TIMEOUT_OFFSET     54U
#define EM_SETTINGS_RETRY_OFFSET       55U
#define EM_SETTINGS_POLL_INTERVAL      56U
#define EM_SETTINGS_STATUS_OFFSET      57U
#define EM_SETTINGS_ERROR_OFFSET       58U
#define EM_SETTINGS_EXCEPTION_OFFSET   59U
#define EM_ADDRESS_TYPE_START         107U
#define EM_ADDRESS_WORD_ORDER_START   113U
#define EM_ADDRESS_SCALE_START        119U
#define APP_MAINS_KWH_START           200U
#define APP_MAINS_KVAH_START          202U
#define APP_DG_KWH_START              204U
#define APP_DG_KVAH_START             206U
#define APP_SOLAR_KWH_START           208U
#define APP_SOLAR_KVAH_START          210U
#define APP_BALANCE_START             212U
#define APP_RELAY_STATE_OFFSET        214U
#define APP_LOW_BALANCE_OFFSET        215U
#define APP_DISCONNECT_REASON_OFFSET  216U
#define APP_STATUS_FLAGS_OFFSET       217U
#define APP_LAST_POLL_TIME_START      218U
#define APP_TAMPER_FLAG_OFFSET        220U
#define APP_TAMPER_TIMES_START        221U
#define CONFIG_EEPROM_OFFSET        0x0100U
#define CONFIG_RECORD_SIZE           86U
#define RTU_FRAME_GAP_MS        4
#define MASTER_TIMEOUT_MS       300
#define MASTER_POLL_INTERVAL_MS 1000
#define METER_READ_CHUNK_SIZE   20
#define MODBUS_MAX_READ_REGISTERS 125
#define CONTROLLER_RX_BUFFER_SIZE 64
#define METER_RX_BUFFER_SIZE    (5 + (METER_READ_CHUNK_SIZE * 2))
#define TX_BUFFER_SIZE          (5 + (MODBUS_MAX_READ_REGISTERS * 2))
#define METER_EEPROM_ADDRESS    0x50
#define METER_EEPROM_OFFSET     0x0020
#define METER_EEPROM_RECORD_SIZE 14
#define METER_EEPROM_SAVE_INTERVAL_MS 60000UL

#define I2C_SI                  0x08
#define I2C_STO                 0x10
#define I2C_STA                 0x20
#define I2C_ENABLE              0x40

static uint8_t xdata port1_rx_buffer[CONTROLLER_RX_BUFFER_SIZE];
static uint8_t xdata port2_rx_buffer[METER_RX_BUFFER_SIZE];
static uint8_t xdata tx_buffer[TX_BUFFER_SIZE];
volatile uint16_t xdata energy_meter_registers[METER_REGISTER_COUNT];
/* Last successfully received meter values; retained across poll timeouts/errors. */
volatile uint16_t xdata s_energy_meter_registers[METER_REGISTER_COUNT];

volatile uint8_t energy_meter_connected;
volatile uint8_t energy_meter_data_valid;
volatile uint16_t energy_meter_successful_responses;
volatile uint8_t energy_meter_master_state;
volatile uint16_t energy_meter_poll_count;
volatile uint16_t energy_meter_request_count;
volatile uint8_t energy_meter_last_error;
volatile uint16_t energy_meter_timeout_count;
volatile uint16_t energy_meter_parity_error_count;
volatile uint16_t energy_meter_invalid_frame_count;
volatile uint8_t energy_meter_last_exception;
volatile uint16_t port1_rx_byte_count;
volatile uint16_t port1_parity_error_count;
volatile uint16_t port1_valid_request_count;
volatile uint16_t port1_response_count;
volatile uint16_t port1_bad_crc_count;
volatile uint16_t port1_wrong_address_count;
volatile uint8_t port1_last_frame_length;
volatile uint8_t xdata port1_last_frame[8];
volatile uint8_t meter_eeprom_present;
volatile uint8_t meter_eeprom_loaded;
volatile uint8_t meter_eeprom_last_error;
volatile uint16_t meter_eeprom_save_count;
static volatile uint8_t port1_rx_length;
static volatile uint8_t port2_rx_length;
static volatile uint8_t port1_silence_ms;
static volatile uint8_t port2_silence_ms;
static volatile uint8_t port1_rx_parity_error;
static volatile uint8_t port2_rx_parity_error;
static volatile uint8_t port1_tx_complete;
static volatile uint8_t port2_tx_complete;
static uint8_t master_waiting;
static uint16_t master_timer_ms;
static uint16_t master_timeout_ms;
#if ENERGY_METER_REGISTER_COUNT <= 4
static uint16_t xdata meter_eeprom_last_saved[METER_REGISTER_COUNT];
#endif
static uint8_t xdata meter_eeprom_record[METER_EEPROM_RECORD_SIZE];
static uint8_t meter_eeprom_save_pending;
static uint8_t meter_eeprom_has_saved_record;
static uint32_t meter_eeprom_save_timer_ms;
static uint16_t meter_read_offset;
static uint8_t meter_read_count;
static uint16_t xdata em_reading_addresses[6];
static uint8_t xdata em_reading_types[6];
static uint8_t xdata em_reading_word_order[6];
static int8_t xdata em_reading_scale[6];
static uint32_t xdata em_decoded_values[7];
static uint32_t xdata pemc_unit_rates[3];
static uint32_t xdata pemc_low_balance_threshold;
static uint8_t xdata pemc_customer_id[10];
static uint8_t pemc_relay_command;
static uint8_t pemc_balance_source;
static uint8_t pemc_relay_state;
static uint8_t pemc_low_balance_alert;
static uint8_t pemc_disconnect_reason;
static uint8_t pemc_tamper_flag;
static uint32_t pemc_last_poll_time;
static uint32_t xdata pemc_tamper_times[5];
static uint16_t pemc_backlight_timeout;
static uint16_t em_response_timeout;
static uint16_t em_retry_count;
static uint16_t em_poll_interval;
static uint16_t pemc_slave_id;
static uint8_t pemc_active_slave_id;
static uint16_t pemc_active_parity;
static uint16_t pemc_parity;
static uint32_t xdata pemc_baud_rate;
static uint16_t em_slave_id;
static uint16_t em_parity;
static uint32_t xdata em_baud_rate;
static uint8_t meter_retry_attempt;
static uint8_t meter_configuration_dirty;
static uint16_t meter_configuration_save_delay;
static uint8_t xdata meter_configuration_record[CONFIG_RECORD_SIZE];

static uint8_t RS485_EvenParity(uint8_t value)
{
    uint8_t parity;

    parity = 0;
    while (value != 0)
    {
        parity ^= (value & 1);
        value >>= 1;
    }
    return parity;
}

static uint8_t RS485_ParityBit(uint8_t value, uint16_t mode)
{
    if (mode == 0) return 1; /* 8N1: ninth-bit slot is the stop-bit level. */
    if (mode == 2) return (uint8_t)!RS485_EvenParity(value);
    return RS485_EvenParity(value);
}

static uint8_t RS485_BaudSupported(uint16_t baud)
{
    return (uint8_t)(baud == 1200 || baud == 2400 || baud == 4800 ||
        baud == 9600 || baud == 19200 || baud == 38400 || baud == 57600 ||
        baud == 115200);
}

static void RS485_SendByte(uint8_t port, uint8_t value)
{
    uint8_t parity;

    parity = RS485_ParityBit(value, port == 1 ? pemc_active_parity : em_parity);
    SFRS = 0;
    if (port == 1)
    {
        port1_tx_complete = 0;
        TB8_1 = parity;
        SBUF_1 = value;
        while (port1_tx_complete == 0) { }
    }
    else
    {
        port2_tx_complete = 0;
        TB8 = parity;
        SBUF = value;
        while (port2_tx_complete == 0) { }
    }
}

static uint16_t Modbus_Crc16(uint8_t xdata *buffer, uint8_t length)
{
    uint8_t index;
    uint8_t crc_shift_count;
    uint16_t crc;

    crc = 0xFFFF;
    for (index = 0; index < length; index++)
    {
        crc ^= buffer[index];
        for (crc_shift_count = 0; crc_shift_count < 8; crc_shift_count++)
        {
            if (crc & 1)
            {
                crc >>= 1;
                crc ^= 0xA001;
            }
            else
            {
                crc >>= 1;
            }
        }
    }

    return crc;
}

static uint8_t MeterEeprom_WaitComplete(void)
{
    uint16_t timeout;

    timeout = 0;
    while ((I2CON & I2C_SI) == 0)
    {
        timeout++;
        if (timeout == 0)
        {
            return 0;
        }
    }
    return 1;
}

static uint8_t MeterEeprom_Start(void)
{
    I2CON |= I2C_STA;
    I2CON &= (uint8_t)~I2C_SI;
    if (!MeterEeprom_WaitComplete()) return 0;
    if (I2STAT != 0x08 && I2STAT != 0x10) return 0;
    I2CON &= (uint8_t)~I2C_STA;
    return 1;
}

static uint8_t MeterEeprom_WriteI2cByte(uint8_t value, uint8_t expected_status)
{
    I2DAT = value;
    I2CON &= (uint8_t)~I2C_SI;
    if (!MeterEeprom_WaitComplete()) return 0;
    return (uint8_t)(I2STAT == expected_status);
}

static uint8_t MeterEeprom_ReadI2cByte(uint8_t acknowledge, uint8_t *value)
{
    if (acknowledge) I2CON |= 0x04;
    else I2CON &= (uint8_t)~0x04;
    I2CON &= (uint8_t)~I2C_SI;
    if (!MeterEeprom_WaitComplete()) return 0;
    if (acknowledge && I2STAT != 0x50) return 0;
    if (!acknowledge && I2STAT != 0x58) return 0;
    *value = I2DAT;
    return 1;
}

static void MeterEeprom_Stop(void)
{
    I2CON |= I2C_STO;
    I2CON &= (uint8_t)~I2C_SI;
}

static uint8_t MeterEeprom_Probe(void)
{
    uint8_t result;

    result = 0;
    if (MeterEeprom_Start())
    {
        result = MeterEeprom_WriteI2cByte((uint8_t)(METER_EEPROM_ADDRESS << 1), 0x18);
    }
    MeterEeprom_Stop();
    return result;
}

static void MeterEeprom_Init(void)
{
    SFRS = 0;
    P13_OPENDRAIN_MODE;
    P14_OPENDRAIN_MODE;
    P1 |= 0x18;
    I2CLK = 59;
    I2CON = I2C_ENABLE;
}

static uint8_t MeterEeprom_ReadRecord(void)
{
    uint8_t index;

    if (!MeterEeprom_Start()) goto error;
    if (!MeterEeprom_WriteI2cByte((uint8_t)(METER_EEPROM_ADDRESS << 1), 0x18)) goto error;
    if (!MeterEeprom_WriteI2cByte((uint8_t)(METER_EEPROM_OFFSET >> 8), 0x28)) goto error;
    if (!MeterEeprom_WriteI2cByte((uint8_t)METER_EEPROM_OFFSET, 0x28)) goto error;
    if (!MeterEeprom_Start()) goto error;
    if (!MeterEeprom_WriteI2cByte((uint8_t)((METER_EEPROM_ADDRESS << 1) | 1), 0x40)) goto error;
    for (index = 0; index < METER_EEPROM_RECORD_SIZE; index++)
    {
        if (!MeterEeprom_ReadI2cByte(index + 1 < METER_EEPROM_RECORD_SIZE,
                                     &meter_eeprom_record[index])) goto error;
    }
    MeterEeprom_Stop();
    return 1;

error:
    MeterEeprom_Stop();
    return 0;
}

static uint8_t MeterEeprom_WriteRecord(void)
{
    uint8_t index;

    for (index = 0; index < METER_EEPROM_RECORD_SIZE; index++)
    {
        if (!MeterEeprom_Start()) goto error;
        if (!MeterEeprom_WriteI2cByte((uint8_t)(METER_EEPROM_ADDRESS << 1), 0x18)) goto error;
        if (!MeterEeprom_WriteI2cByte((uint8_t)(METER_EEPROM_OFFSET >> 8), 0x28)) goto error;
        if (!MeterEeprom_WriteI2cByte((uint8_t)(METER_EEPROM_OFFSET + index), 0x28)) goto error;
        if (!MeterEeprom_WriteI2cByte(meter_eeprom_record[index], 0x28)) goto error;
        MeterEeprom_Stop();
        Board_DelayMs(5);
    }
    return 1;

error:
    MeterEeprom_Stop();
    return 0;
}

static void MeterEeprom_BuildRecord(void)
{
    uint8_t index;
    uint16_t crc;

    meter_eeprom_record[0] = 0x4D;
    meter_eeprom_record[1] = 0x52;
    meter_eeprom_record[2] = 1;
    meter_eeprom_record[3] = METER_REGISTER_COUNT;
    for (index = 0; index < METER_REGISTER_COUNT; index++)
    {
        meter_eeprom_record[4 + index * 2] = (uint8_t)(s_energy_meter_registers[index] >> 8);
        meter_eeprom_record[5 + index * 2] = (uint8_t)s_energy_meter_registers[index];
    }
    crc = Modbus_Crc16(meter_eeprom_record, 12);
    meter_eeprom_record[12] = (uint8_t)crc;
    meter_eeprom_record[13] = (uint8_t)(crc >> 8);
}

#if ENERGY_METER_REGISTER_COUNT <= 4
static void MeterEeprom_Load(void)
{
    uint8_t index;
    uint16_t crc;

    meter_eeprom_present = MeterEeprom_Probe();
    if (!meter_eeprom_present) return;
    if (!MeterEeprom_ReadRecord())
    {
        meter_eeprom_last_error = 1;
        return;
    }
    crc = Modbus_Crc16(meter_eeprom_record, 12);
    if (meter_eeprom_record[0] != 0x4D || meter_eeprom_record[1] != 0x52 ||
        meter_eeprom_record[2] != 1 || meter_eeprom_record[3] != METER_REGISTER_COUNT ||
        meter_eeprom_record[12] != (uint8_t)crc ||
        meter_eeprom_record[13] != (uint8_t)(crc >> 8))
    {
        meter_eeprom_last_error = 2;
        return;
    }
    for (index = 0; index < METER_REGISTER_COUNT; index++)
    {
        s_energy_meter_registers[index] =
            ((uint16_t)meter_eeprom_record[4 + index * 2] << 8) |
            meter_eeprom_record[5 + index * 2];
        energy_meter_registers[index] = s_energy_meter_registers[index];
        meter_eeprom_last_saved[index] = s_energy_meter_registers[index];
    }
    meter_eeprom_has_saved_record = 1;
    meter_eeprom_loaded = 1;
}
#endif

static void MeterEeprom_RequestSave(void)
{
#if ENERGY_METER_REGISTER_COUNT <= 4
    uint8_t index;

    for (index = 0; index < METER_REGISTER_COUNT; index++)
    {
        if (!meter_eeprom_has_saved_record ||
            s_energy_meter_registers[index] != meter_eeprom_last_saved[index])
        {
            meter_eeprom_save_pending = 1;
            if (!meter_eeprom_has_saved_record)
            {
                meter_eeprom_save_timer_ms = METER_EEPROM_SAVE_INTERVAL_MS;
            }
            return;
        }
    }
#endif
}

static void MeterEeprom_Service(void)
{
#if ENERGY_METER_REGISTER_COUNT <= 4
    uint8_t index;

    if (!meter_eeprom_present || !meter_eeprom_save_pending ||
        meter_eeprom_save_timer_ms < METER_EEPROM_SAVE_INTERVAL_MS)
    {
        return;
    }
    MeterEeprom_BuildRecord();
    if (!MeterEeprom_WriteRecord())
    {
        meter_eeprom_last_error = 3;
        meter_eeprom_save_timer_ms = 0;
        return;
    }
    for (index = 0; index < METER_REGISTER_COUNT; index++)
    {
        meter_eeprom_last_saved[index] = s_energy_meter_registers[index];
    }
    meter_eeprom_has_saved_record = 1;
    meter_eeprom_save_pending = 0;
    meter_eeprom_save_timer_ms = 0;
    meter_eeprom_last_error = 0;
    if (meter_eeprom_save_count != 0xFFFF)
    {
        meter_eeprom_save_count++;
    }
#endif
}

static uint8_t Modbus_FrameValid(uint8_t xdata *frame, uint8_t length)
{
    uint16_t crc;

    if (length < 4)
    {
        return 0;
    }
    crc = Modbus_Crc16(frame, length - 2);
    return (frame[length - 2] == (uint8_t)crc &&
            frame[length - 1] == (uint8_t)(crc >> 8));
}

static void RS485_ClearMeterRegisters(void)
{
    uint8_t index;

    for (index = 0; index < METER_REGISTER_COUNT; index++)
    {
        energy_meter_registers[index] = 0;
    }
}

void RS485_UART0_ISR(void) interrupt 4
{
    uint8_t sfr_page;
    uint8_t value;

    sfr_page = SFRS;
    SFRS = 0;
    if (RI)
    {
        value = SBUF;
        RI = 0;
        if (em_parity != 0 && RB8 != RS485_ParityBit(value, em_parity))
        {
            port2_rx_parity_error = 1;
        }
        if (port2_rx_length < METER_RX_BUFFER_SIZE)
        {
            port2_rx_buffer[port2_rx_length++] = value;
            port2_silence_ms = 0;
        }
    }
    if (TI)
    {
        TI = 0;
        port2_tx_complete = 1;
    }
    SFRS = sfr_page;
}

void RS485_UART1_ISR(void) interrupt 15
{
    uint8_t sfr_page;
    uint8_t value;

    sfr_page = SFRS;
    SFRS = 0;
    if (RI_1)
    {
        value = SBUF_1;
        RI_1 = 0;
        if (pemc_active_parity != 0 &&
            RB8_1 != RS485_ParityBit(value, pemc_active_parity))
        {
            port1_rx_parity_error = 1;
            if (port1_parity_error_count != 0xFFFF)
            {
                port1_parity_error_count++;
            }
        }
        if (port1_rx_byte_count != 0xFFFF)
        {
            port1_rx_byte_count++;
        }
        if (port1_rx_length < CONTROLLER_RX_BUFFER_SIZE)
        {
            port1_rx_buffer[port1_rx_length++] = value;
            port1_silence_ms = 0;
        }
    }
    if (TI_1)
    {
        TI_1 = 0;
        port1_tx_complete = 1;
    }
    SFRS = sfr_page;
}

static void RS485_SendFrame(uint8_t port, uint8_t xdata *frame, uint8_t length)
{
    uint8_t index;
    uint16_t crc;

    crc = Modbus_Crc16(frame, length);
    frame[length] = (uint8_t)crc;
    frame[length + 1] = (uint8_t)(crc >> 8);

    Board_SetRs485Transmit(port, 1);
    Board_DelayMs(1);
    for (index = 0; index < (uint8_t)(length + 2); index++)
    {
        RS485_SendByte(port, frame[index]);
    }
    Board_DelayMs(2);
    Board_SetRs485Transmit(port, 0);
}

static void Modbus_SendException(uint8_t function, uint8_t exception_code)
{
    tx_buffer[0] = pemc_active_slave_id;
    tx_buffer[1] = function | 0x80;
    tx_buffer[2] = exception_code;
    RS485_SendFrame(1, tx_buffer, 3);
}

static uint16_t Modbus_ReadU32Word(uint32_t value, uint8_t low_word)
{
    return low_word ? (uint16_t)value : (uint16_t)(value >> 16);
}

static uint8_t Modbus_WriteU32Word(uint32_t xdata *value, uint16_t address,
                                   uint16_t start, uint16_t word)
{
    if (address == start)
    {
        *value = (*value & 0x0000FFFFUL) | ((uint32_t)word << 16);
        return 1;
    }
    if (address == start + 1)
    {
        *value = (*value & 0xFFFF0000UL) | word;
        return 1;
    }
    return 0;
}

/* Return Modbus exception 0 when valid, 2 for address, or 3 for value. */
static uint8_t Modbus_ValidateWrite(uint16_t address, uint16_t value)
{
    if (address == PEMC_CONTROLLER_ID_OFFSET)
        return (value >= 1 && value <= 247) ? 0 : 3;
    if (address == PEMC_PARITY_OFFSET || address == EM_SETTINGS_PARITY_OFFSET)
        return (value <= 2) ? 0 : 3;
    if (address >= PEMC_BAUD_START && address <= PEMC_BAUD_START + 1)
        return ((address == PEMC_BAUD_START && value == 0) ||
                (address == PEMC_BAUD_START + 1 && RS485_BaudSupported(value))) ? 0 : 3;
    if (address >= EM_SETTINGS_BAUD_START && address <= EM_SETTINGS_BAUD_START + 1)
        return ((address == EM_SETTINGS_BAUD_START && value == 0) ||
                (address == EM_SETTINGS_BAUD_START + 1 && RS485_BaudSupported(value))) ? 0 : 3;
    if (address == PEMC_BACKLIGHT_TIMEOUT_OFFSET ||
        address == EM_SETTINGS_TIMEOUT_OFFSET)
        return (address == PEMC_BACKLIGHT_TIMEOUT_OFFSET || value != 0) ? 0 : 3;
    if (address >= PEMC_RTC_YEAR_OFFSET && address <= PEMC_RTC_SECOND_OFFSET)
        return 0;
    if ((address >= PEMC_MAINS_RATE_START && address <= PEMC_MAINS_RATE_START + 1) ||
        (address >= PEMC_DG_RATE_START && address <= PEMC_DG_RATE_START + 1) ||
        (address >= PEMC_SOLAR_RATE_START && address <= PEMC_SOLAR_RATE_START + 1) ||
        (address >= PEMC_LOW_BALANCE_START && address <= PEMC_LOW_BALANCE_START + 1))
        return 0;
    if (address == PEMC_RELAY_COMMAND_OFFSET ||
        address == PEMC_BALANCE_SOURCE_OFFSET)
        return (value <= (address == PEMC_RELAY_COMMAND_OFFSET ? 1 : 2)) ? 0 : 3;
    if (address >= PEMC_CUSTOMER_ID_START && address <= PEMC_SETTINGS_END)
        return 0;
    if (address == EM_SETTINGS_SLAVE_ID_OFFSET)
        return (value >= 1 && value <= 247) ? 0 : 3;
    if (address == EM_SETTINGS_RETRY_OFFSET)
        return (value <= 10) ? 0 : 3;
    if (address == EM_SETTINGS_POLL_INTERVAL)
        return (value >= 1 && value <= 60) ? 0 : 3;
    if (address >= ENERGY_METER_ADDRESS_START &&
        address <= ENERGY_METER_ADDRESS_START + 5)
        return (value == 0xFFFF ||
                (value >= METER_REGISTER_START &&
                 value < METER_REGISTER_START + METER_REGISTER_COUNT)) ? 0 : 3;
    if (address >= EM_ADDRESS_TYPE_START && address <= EM_ADDRESS_TYPE_START + 5)
        return (value <= 2) ? 0 : 3;
    if (address >= EM_ADDRESS_WORD_ORDER_START &&
        address <= EM_ADDRESS_WORD_ORDER_START + 5)
        return (value <= 1) ? 0 : 3;
    if (address >= EM_ADDRESS_SCALE_START && address <= EM_ADDRESS_SCALE_START + 5)
    {
        int16_t signed_value;
        signed_value = (int16_t)value;
        return (signed_value >= -5 && signed_value <= 5) ? 0 : 3;
    }
    if (address >= APP_MAINS_KWH_START && address <= APP_TAMPER_TIMES_START + 9)
        return 2;
    if ((address >= PEMC_SETTINGS_START && address <= PEMC_SETTINGS_END) ||
        (address >= ENERGY_METER_SETTINGS_START && address <= ENERGY_METER_SETTINGS_END) ||
        (address >= ENERGY_METER_ADDRESS_START && address <= ENERGY_METER_ADDRESS_END))
        return 2;
    return 2;
}

static uint8_t Modbus_WriteHoldingRegister(uint16_t address, uint16_t value)
{
    uint8_t index;
    RTC_DateTime candidate;

    if (Modbus_ValidateWrite(address, value) != 0)
        return Modbus_ValidateWrite(address, value);
    if (address == PEMC_CONTROLLER_ID_OFFSET)
        pemc_slave_id = value;
    else if (address == PEMC_PARITY_OFFSET)
        pemc_parity = value;
    else if (address >= PEMC_BAUD_START && address <= PEMC_BAUD_START + 1)
        Modbus_WriteU32Word(&pemc_baud_rate, address, PEMC_BAUD_START, value);
    else if (address == PEMC_BACKLIGHT_TIMEOUT_OFFSET)
        pemc_backlight_timeout = value;
    else if (address >= PEMC_RTC_YEAR_OFFSET && address <= PEMC_RTC_SECOND_OFFSET)
    {
        candidate.year = rtc_datetime.year;
        candidate.month = rtc_datetime.month;
        candidate.day = rtc_datetime.day;
        candidate.weekday = rtc_datetime.weekday;
        candidate.hours = rtc_datetime.hours;
        candidate.minutes = rtc_datetime.minutes;
        candidate.seconds = rtc_datetime.seconds;
        switch (address)
        {
            case PEMC_RTC_YEAR_OFFSET: candidate.year = value; break;
            case PEMC_RTC_MONTH_OFFSET: candidate.month = (uint8_t)value; break;
            case PEMC_RTC_DAY_OFFSET: candidate.day = (uint8_t)value; break;
            case PEMC_RTC_HOUR_OFFSET: candidate.hours = (uint8_t)value; break;
            case PEMC_RTC_MINUTE_OFFSET: candidate.minutes = (uint8_t)value; break;
            default: candidate.seconds = (uint8_t)value; break;
        }
        candidate.weekday = rtc_datetime.weekday;
        if (!RTC_WriteDateTime(&candidate)) return 3;
        rtc_datetime.year = candidate.year;
        rtc_datetime.month = candidate.month;
        rtc_datetime.day = candidate.day;
        rtc_datetime.hours = candidate.hours;
        rtc_datetime.minutes = candidate.minutes;
        rtc_datetime.seconds = candidate.seconds;
        RTC_SaveDateTimeToEEPROM(&candidate);
    }
    else if (address >= PEMC_MAINS_RATE_START && address <= PEMC_MAINS_RATE_START + 1)
        Modbus_WriteU32Word(&pemc_unit_rates[0], address, PEMC_MAINS_RATE_START, value);
    else if (address >= PEMC_DG_RATE_START && address <= PEMC_DG_RATE_START + 1)
        Modbus_WriteU32Word(&pemc_unit_rates[1], address, PEMC_DG_RATE_START, value);
    else if (address >= PEMC_SOLAR_RATE_START && address <= PEMC_SOLAR_RATE_START + 1)
        Modbus_WriteU32Word(&pemc_unit_rates[2], address, PEMC_SOLAR_RATE_START, value);
    else if (address >= PEMC_LOW_BALANCE_START && address <= PEMC_LOW_BALANCE_START + 1)
        Modbus_WriteU32Word(&pemc_low_balance_threshold, address,
                            PEMC_LOW_BALANCE_START, value);
    else if (address == PEMC_RELAY_COMMAND_OFFSET)
    {
        pemc_relay_command = (uint8_t)value;
        Board_PulseRelay(1, (uint8_t)value, 100);
        pemc_relay_state = (uint8_t)value;
        pemc_disconnect_reason = value ? 0 : 2;
    }
    else if (address == PEMC_BALANCE_SOURCE_OFFSET)
        pemc_balance_source = (uint8_t)value;
    else if (address >= PEMC_CUSTOMER_ID_START && address <= PEMC_SETTINGS_END)
    {
        index = (uint8_t)((address - PEMC_CUSTOMER_ID_START) * 2);
        pemc_customer_id[index] = (uint8_t)(value >> 8);
        pemc_customer_id[index + 1] = (uint8_t)value;
    }
    else if (address == EM_SETTINGS_SLAVE_ID_OFFSET)
        em_slave_id = value;
    else if (address == EM_SETTINGS_PARITY_OFFSET)
        em_parity = value;
    else if (address >= EM_SETTINGS_BAUD_START && address <= EM_SETTINGS_BAUD_START + 1)
        Modbus_WriteU32Word(&em_baud_rate, address, EM_SETTINGS_BAUD_START, value);
    else if (address == EM_SETTINGS_TIMEOUT_OFFSET)
        em_response_timeout = value;
    else if (address == EM_SETTINGS_RETRY_OFFSET)
        em_retry_count = value;
    else if (address == EM_SETTINGS_POLL_INTERVAL)
        em_poll_interval = value;
    else if (address >= ENERGY_METER_ADDRESS_START &&
             address <= ENERGY_METER_ADDRESS_START + 5)
        em_reading_addresses[address - ENERGY_METER_ADDRESS_START] = value;
    else if (address >= EM_ADDRESS_TYPE_START && address <= EM_ADDRESS_TYPE_START + 5)
        em_reading_types[address - EM_ADDRESS_TYPE_START] = (uint8_t)value;
    else if (address >= EM_ADDRESS_WORD_ORDER_START &&
             address <= EM_ADDRESS_WORD_ORDER_START + 5)
        em_reading_word_order[address - EM_ADDRESS_WORD_ORDER_START] = (uint8_t)value;
    else if (address >= EM_ADDRESS_SCALE_START && address <= EM_ADDRESS_SCALE_START + 5)
        em_reading_scale[address - EM_ADDRESS_SCALE_START] = (int8_t)value;
    else
        return 2;
    meter_configuration_dirty = 1;
    meter_configuration_save_delay = 100;
    return 0;
}

static uint32_t EM_ReadConfiguredValue(uint8_t reading)
{
    uint16_t pdu_address;
    uint16_t word0;
    uint16_t word1;
    uint32_t value;
    uint8_t type;
    int8_t scale;
    uint8_t count;
    union
    {
        uint32_t bits;
        float real;
    } float_value;

    pdu_address = em_reading_addresses[reading];
    type = em_reading_types[reading];
    scale = em_reading_scale[reading];
    if (pdu_address == 0xFFFF || pdu_address < METER_REGISTER_START ||
        pdu_address >= (METER_REGISTER_START + METER_REGISTER_COUNT) ||
        type > 2 || scale < -5 || scale > 5)
    {
        return 0;
    }

    word0 = s_energy_meter_registers[pdu_address - METER_REGISTER_START];
    if (type == 0)
    {
        value = word0;
    }
    else
    {
        if ((pdu_address - METER_REGISTER_START + 1) >= METER_REGISTER_COUNT)
            return 0;
        word1 = s_energy_meter_registers[pdu_address - METER_REGISTER_START + 1];
        if (em_reading_word_order[reading] == 1)
        {
            value = ((uint32_t)word1 << 16) | word0;
        }
        else
        {
            value = ((uint32_t)word0 << 16) | word1;
        }
        if (type == 2)
        {
            float_value.bits = value;
            if (float_value.real < 0.0f || float_value.real > 4294967040.0f)
                return 0;
            if (scale > 0)
            {
                while (scale-- > 0) float_value.real *= 10.0f;
            }
            else
            {
                while (scale++ < 0) float_value.real /= 10.0f;
            }
            if (float_value.real < 0.0f || float_value.real > 4294967040.0f)
                return 0;
            return (uint32_t)float_value.real;
        }
    }

    count = (uint8_t)(scale > 0 ? scale : -scale);
    while (count--)
    {
        if (scale > 0)
        {
            if (value > 429496729UL) return 0xFFFFFFFFUL;
            value *= 10UL;
        }
        else
        {
            value /= 10UL;
        }
    }
    return value;
}

static void EM_UpdateDecodedValues(void)
{
    uint8_t reading;

    for (reading = 0; reading < 6; reading++)
    {
        em_decoded_values[reading] = EM_ReadConfiguredValue(reading);
    }
}

static uint8_t PEMC_IsLeapYear(uint16_t year)
{
    return (uint8_t)(((year % 4U) == 0U && (year % 100U) != 0U) ||
                     (year % 400U) == 0U);
}

static uint32_t PEMC_RtcTimestamp(void)
{
    uint16_t year;
    uint8_t month;
    uint8_t days_in_month;
    uint32_t days;

    if (!rtc_datetime_valid || rtc_datetime.year < 2000 ||
        rtc_datetime.year > 2136 || rtc_datetime.month < 1 ||
        rtc_datetime.month > 12 || rtc_datetime.day < 1 ||
        rtc_datetime.day > 31 || rtc_datetime.hours > 23 ||
        rtc_datetime.minutes > 59 || rtc_datetime.seconds > 59)
        return 0;
    days = 0;
    for (year = 2000; year < rtc_datetime.year; year++)
        days += PEMC_IsLeapYear(year) ? 366UL : 365UL;
    for (month = 1; month < rtc_datetime.month; month++)
    {
        if (month == 2) days_in_month = PEMC_IsLeapYear(rtc_datetime.year) ? 29 : 28;
        else if (month == 4 || month == 6 || month == 9 || month == 11)
            days_in_month = 30;
        else days_in_month = 31;
        days += days_in_month;
    }
    days += (uint32_t)(rtc_datetime.day - 1);
    return days * 86400UL + (uint32_t)rtc_datetime.hours * 3600UL +
           (uint32_t)rtc_datetime.minutes * 60UL + rtc_datetime.seconds;
}

static uint16_t Modbus_ReadHoldingRegister(uint16_t address)
{
    uint8_t index;
    uint8_t flags;

    if (address >= RAW_ENERGY_METER_START &&
        address < SLAVE_HOLDING_REGISTER_COUNT)
    {
        return s_energy_meter_registers[address - RAW_ENERGY_METER_START];
    }
    if (address >= PEMC_BAUD_START && address <= PEMC_BAUD_START + 1)
    {
        return Modbus_ReadU32Word(pemc_baud_rate,
                                  (uint8_t)(address - PEMC_BAUD_START));
    }
    if ((address >= PEMC_MAINS_RATE_START && address <= PEMC_MAINS_RATE_START + 1) ||
        (address >= PEMC_DG_RATE_START && address <= PEMC_DG_RATE_START + 1) ||
        (address >= PEMC_SOLAR_RATE_START && address <= PEMC_SOLAR_RATE_START + 1))
    {
        index = (uint8_t)((address - PEMC_MAINS_RATE_START) / 2);
        return Modbus_ReadU32Word(pemc_unit_rates[index],
            (uint8_t)(address - (PEMC_MAINS_RATE_START + index * 2)));
    }
    if (address >= PEMC_LOW_BALANCE_START && address <= PEMC_LOW_BALANCE_START + 1)
    {
        return Modbus_ReadU32Word(pemc_low_balance_threshold,
                                  (uint8_t)(address - PEMC_LOW_BALANCE_START));
    }
    if (address >= PEMC_CUSTOMER_ID_START && address <= PEMC_SETTINGS_END)
    {
        index = (uint8_t)((address - PEMC_CUSTOMER_ID_START) * 2);
        return (uint16_t)(((uint16_t)pemc_customer_id[index] << 8) |
                          pemc_customer_id[index + 1]);
    }
    if (address >= EM_SETTINGS_BAUD_START && address <= EM_SETTINGS_BAUD_START + 1)
    {
        return Modbus_ReadU32Word(em_baud_rate,
                                  (uint8_t)(address - EM_SETTINGS_BAUD_START));
    }
    if (address >= ENERGY_METER_ADDRESS_START &&
        address <= ENERGY_METER_ADDRESS_START + 5)
    {
        return em_reading_addresses[address - ENERGY_METER_ADDRESS_START];
    }
    if (address >= EM_ADDRESS_TYPE_START && address <= EM_ADDRESS_TYPE_START + 5)
    {
        return em_reading_types[address - EM_ADDRESS_TYPE_START];
    }
    if (address >= EM_ADDRESS_WORD_ORDER_START &&
        address <= EM_ADDRESS_WORD_ORDER_START + 5)
    {
        return em_reading_word_order[address - EM_ADDRESS_WORD_ORDER_START];
    }
    if (address >= EM_ADDRESS_SCALE_START && address <= EM_ADDRESS_SCALE_START + 5)
    {
        return (uint16_t)(int16_t)em_reading_scale[address - EM_ADDRESS_SCALE_START];
    }
    if (address >= APP_MAINS_KWH_START && address <= APP_MAINS_KWH_START + 1)
        return Modbus_ReadU32Word(em_decoded_values[0], (uint8_t)(address - APP_MAINS_KWH_START));
    if (address >= APP_MAINS_KVAH_START && address <= APP_MAINS_KVAH_START + 1)
        return Modbus_ReadU32Word(em_decoded_values[1], (uint8_t)(address - APP_MAINS_KVAH_START));
    if (address >= APP_DG_KWH_START && address <= APP_DG_KWH_START + 1)
        return Modbus_ReadU32Word(em_decoded_values[2], (uint8_t)(address - APP_DG_KWH_START));
    if (address >= APP_DG_KVAH_START && address <= APP_DG_KVAH_START + 1)
        return Modbus_ReadU32Word(em_decoded_values[3], (uint8_t)(address - APP_DG_KVAH_START));
    if (address >= APP_SOLAR_KWH_START && address <= APP_SOLAR_KWH_START + 1)
        return Modbus_ReadU32Word(em_decoded_values[4], (uint8_t)(address - APP_SOLAR_KWH_START));
    if (address >= APP_SOLAR_KVAH_START && address <= APP_SOLAR_KVAH_START + 1)
        return Modbus_ReadU32Word(em_decoded_values[5], (uint8_t)(address - APP_SOLAR_KVAH_START));
    if (address >= APP_BALANCE_START && address <= APP_BALANCE_START + 1)
        return Modbus_ReadU32Word(em_decoded_values[6], (uint8_t)(address - APP_BALANCE_START));
    if (address >= APP_LAST_POLL_TIME_START && address <= APP_LAST_POLL_TIME_START + 1)
        return Modbus_ReadU32Word(pemc_last_poll_time,
            (uint8_t)(address - APP_LAST_POLL_TIME_START));
    if (address >= APP_TAMPER_TIMES_START && address <= APP_TAMPER_TIMES_START + 9)
    {
        index = (uint8_t)((address - APP_TAMPER_TIMES_START) / 2);
        return Modbus_ReadU32Word(pemc_tamper_times[index],
            (uint8_t)(address - (APP_TAMPER_TIMES_START + index * 2)));
    }

    switch (address)
    {
        case PEMC_CONTROLLER_ID_OFFSET: return pemc_slave_id;
        case PEMC_PARITY_OFFSET: return pemc_parity;
        case PEMC_BACKLIGHT_TIMEOUT_OFFSET: return pemc_backlight_timeout;
        case PEMC_RTC_YEAR_OFFSET: return rtc_datetime.year;
        case PEMC_RTC_MONTH_OFFSET: return rtc_datetime.month;
        case PEMC_RTC_DAY_OFFSET: return rtc_datetime.day;
        case PEMC_RTC_HOUR_OFFSET: return rtc_datetime.hours;
        case PEMC_RTC_MINUTE_OFFSET: return rtc_datetime.minutes;
        case PEMC_RTC_SECOND_OFFSET: return rtc_datetime.seconds;
        case PEMC_RELAY_COMMAND_OFFSET: return pemc_relay_command;
        case PEMC_BALANCE_SOURCE_OFFSET: return pemc_balance_source;
        case EM_SETTINGS_SLAVE_ID_OFFSET: return em_slave_id;
        case EM_SETTINGS_PARITY_OFFSET: return em_parity;
        case EM_SETTINGS_TIMEOUT_OFFSET: return em_response_timeout;
        case EM_SETTINGS_RETRY_OFFSET: return em_retry_count;
        case EM_SETTINGS_POLL_INTERVAL: return em_poll_interval;
        case EM_SETTINGS_STATUS_OFFSET:
            return energy_meter_connected ? 1 :
                   ((energy_meter_request_count &&
                     energy_meter_last_error != ENERGY_METER_ERROR_NONE) ? 2 : 0);
        case EM_SETTINGS_ERROR_OFFSET: return energy_meter_last_error;
        case EM_SETTINGS_EXCEPTION_OFFSET: return energy_meter_last_exception;
        case APP_RELAY_STATE_OFFSET: return pemc_relay_state;
        case APP_LOW_BALANCE_OFFSET: return pemc_low_balance_alert;
        case APP_DISCONNECT_REASON_OFFSET: return pemc_disconnect_reason;
        case APP_STATUS_FLAGS_OFFSET:
            flags = 0;
            if (energy_meter_connected) flags |= 0x01;
            if (energy_meter_data_valid) flags |= 0x02;
            if (port1_response_count) flags |= 0x04;
            if (rtc_datetime_valid) flags |= 0x08;
            return flags;
        case APP_TAMPER_FLAG_OFFSET: return pemc_tamper_flag;
        default: return 0;
    }
}

static void Modbus_HandleSlaveFrame(uint8_t length)
{
    uint8_t function;
    uint8_t quantity;
    uint8_t index;
    uint8_t exception_code;
    uint8_t byte_count;
    uint16_t start_address;
    uint16_t value;
    uint8_t rtc_in_block;
    RTC_DateTime rtc_candidate;

    if (!Modbus_FrameValid(port1_rx_buffer, length))
    {
        if (port1_bad_crc_count != 0xFFFF)
        {
            port1_bad_crc_count++;
        }
        return;
    }
    if (port1_rx_buffer[0] != pemc_active_slave_id)
    {
        if (port1_wrong_address_count != 0xFFFF)
        {
            port1_wrong_address_count++;
        }
        return;
    }

    if (port1_valid_request_count != 0xFFFF)
    {
        port1_valid_request_count++;
    }

    function = port1_rx_buffer[1];
    start_address = ((uint16_t)port1_rx_buffer[2] << 8) |
                    port1_rx_buffer[3];

    if (function == 0x06)
    {
        if (length != 8)
        {
            Modbus_SendException(function, 0x03);
            return;
        }
        value = ((uint16_t)port1_rx_buffer[4] << 8) | port1_rx_buffer[5];
        exception_code = Modbus_ValidateWrite(start_address, value);
        if (exception_code != 0)
        {
            Modbus_SendException(function, exception_code);
            return;
        }
        exception_code = Modbus_WriteHoldingRegister(start_address, value);
        if (exception_code != 0)
        {
            Modbus_SendException(function, exception_code);
            return;
        }
        for (index = 0; index < 6; index++) tx_buffer[index] = port1_rx_buffer[index];
        RS485_SendFrame(1, tx_buffer, 6);
        if (port1_response_count != 0xFFFF) port1_response_count++;
        return;
    }
    if (function == 0x10)
    {
        if (length < 11)
        {
            Modbus_SendException(function, 0x03);
            return;
        }
        quantity = (uint8_t)(((uint16_t)port1_rx_buffer[4] << 8) |
                             port1_rx_buffer[5]);
        byte_count = port1_rx_buffer[6];
        if (port1_rx_buffer[4] != 0 || quantity == 0 || quantity > 25 ||
            byte_count != quantity * 2 ||
            length != (uint8_t)(9 + byte_count) ||
            (uint16_t)(start_address + quantity) > SLAVE_REGISTER_COUNT)
        {
            Modbus_SendException(function, 0x03);
            return;
        }
        for (index = 0; index < quantity; index++)
        {
            value = ((uint16_t)port1_rx_buffer[7 + index * 2] << 8) |
                    port1_rx_buffer[8 + index * 2];
            exception_code = Modbus_ValidateWrite(start_address + index, value);
            if (exception_code != 0)
            {
                Modbus_SendException(function, exception_code);
                return;
            }
        }
        rtc_in_block = (start_address <= PEMC_RTC_SECOND_OFFSET &&
            (uint16_t)(start_address + quantity) > PEMC_RTC_YEAR_OFFSET);
        if (rtc_in_block)
        {
            rtc_candidate.year = rtc_datetime.year;
            rtc_candidate.month = rtc_datetime.month;
            rtc_candidate.day = rtc_datetime.day;
            rtc_candidate.weekday = rtc_datetime.weekday;
            rtc_candidate.hours = rtc_datetime.hours;
            rtc_candidate.minutes = rtc_datetime.minutes;
            rtc_candidate.seconds = rtc_datetime.seconds;
            for (index = 0; index < quantity; index++)
            {
                uint16_t field_address;
                field_address = start_address + index;
                value = ((uint16_t)port1_rx_buffer[7 + index * 2] << 8) |
                        port1_rx_buffer[8 + index * 2];
                switch (field_address)
                {
                    case PEMC_RTC_YEAR_OFFSET: rtc_candidate.year = value; break;
                    case PEMC_RTC_MONTH_OFFSET: rtc_candidate.month = (uint8_t)value; break;
                    case PEMC_RTC_DAY_OFFSET: rtc_candidate.day = (uint8_t)value; break;
                    case PEMC_RTC_HOUR_OFFSET: rtc_candidate.hours = (uint8_t)value; break;
                    case PEMC_RTC_MINUTE_OFFSET: rtc_candidate.minutes = (uint8_t)value; break;
                    case PEMC_RTC_SECOND_OFFSET: rtc_candidate.seconds = (uint8_t)value; break;
                }
            }
            if (!RTC_WriteDateTime(&rtc_candidate))
            {
                Modbus_SendException(function, 0x03);
                return;
            }
            rtc_datetime.year = rtc_candidate.year;
            rtc_datetime.month = rtc_candidate.month;
            rtc_datetime.day = rtc_candidate.day;
            rtc_datetime.hours = rtc_candidate.hours;
            rtc_datetime.minutes = rtc_candidate.minutes;
            rtc_datetime.seconds = rtc_candidate.seconds;
            RTC_SaveDateTimeToEEPROM(&rtc_candidate);
        }
        for (index = 0; index < quantity; index++)
        {
            if (rtc_in_block && start_address + index >= PEMC_RTC_YEAR_OFFSET &&
                start_address + index <= PEMC_RTC_SECOND_OFFSET)
                continue;
            value = ((uint16_t)port1_rx_buffer[7 + index * 2] << 8) |
                    port1_rx_buffer[8 + index * 2];
            exception_code = Modbus_WriteHoldingRegister(start_address + index, value);
            if (exception_code != 0)
            {
                Modbus_SendException(function, exception_code);
                return;
            }
        }
        tx_buffer[0] = pemc_active_slave_id;
        tx_buffer[1] = function;
        tx_buffer[2] = (uint8_t)(start_address >> 8);
        tx_buffer[3] = (uint8_t)start_address;
        tx_buffer[4] = (uint8_t)(quantity >> 8);
        tx_buffer[5] = quantity;
        RS485_SendFrame(1, tx_buffer, 6);
        if (port1_response_count != 0xFFFF) port1_response_count++;
        return;
    }
    if (function != 0x03)
    {
        Modbus_SendException(function, 0x01);
        return;
    }
    if (length != 8) return;
    if (port1_rx_buffer[4] != 0)
    {
        Modbus_SendException(function, 0x03);
        return;
    }
    quantity = port1_rx_buffer[5];
    if (quantity == 0 || quantity > MODBUS_MAX_READ_REGISTERS ||
        quantity > SLAVE_REGISTER_COUNT ||
        start_address >= SLAVE_REGISTER_COUNT ||
        (uint16_t)(start_address + quantity) > SLAVE_REGISTER_COUNT)
    {
        Modbus_SendException(function, 0x02);
        return;
    }

    tx_buffer[0] = pemc_active_slave_id;
    tx_buffer[1] = function;
    tx_buffer[2] = quantity * 2;
    for (index = 0; index < quantity; index++)
    {
        value = Modbus_ReadHoldingRegister(start_address + index);
        tx_buffer[3 + (index * 2)] = (uint8_t)(value >> 8);
        tx_buffer[4 + (index * 2)] = (uint8_t)value;
    }
    RS485_SendFrame(1, tx_buffer, (uint8_t)(3 + quantity * 2));
    if (port1_response_count != 0xFFFF)
    {
        port1_response_count++;
    }
}

static void RS485_StartMeterRead(void)
{
    uint16_t remaining;

    remaining = METER_REGISTER_COUNT - meter_read_offset;
    meter_read_count = (remaining > METER_READ_CHUNK_SIZE) ?
                       METER_READ_CHUNK_SIZE : (uint8_t)remaining;
    tx_buffer[0] = (uint8_t)em_slave_id;
    tx_buffer[1] = 0x03;
    tx_buffer[2] = (uint8_t)((METER_REGISTER_START + meter_read_offset) >> 8);
    tx_buffer[3] = (uint8_t)(METER_REGISTER_START + meter_read_offset);
    tx_buffer[4] = 0;
    tx_buffer[5] = meter_read_count;
    port2_rx_length = 0;
    port2_rx_parity_error = 0;
    port2_silence_ms = 0;
    RS485_SendFrame(2, tx_buffer, 6);
    master_waiting = 1;
    energy_meter_master_state = 1;
    if (energy_meter_request_count != 0xFFFF)
    {
        energy_meter_request_count++;
    }
    master_timeout_ms = 0;
}

static void RS485_HandleMeterResponse(uint8_t length)
{
    uint8_t index;
    uint8_t expected_length;

    if (!master_waiting)
    {
        return;
    }
    if (!Modbus_FrameValid(port2_rx_buffer, length) ||
        port2_rx_buffer[0] != (uint8_t)em_slave_id)
    {
        energy_meter_last_error = ENERGY_METER_ERROR_BAD_FRAME;
        if (energy_meter_invalid_frame_count != 0xFFFF)
        {
            energy_meter_invalid_frame_count++;
        }
        return;
    }
    if (port2_rx_buffer[1] & 0x80)
    {
        energy_meter_last_error = ENERGY_METER_ERROR_EXCEPTION;
        energy_meter_last_exception = port2_rx_buffer[2];
        energy_meter_connected = 0;
        energy_meter_data_valid = 0;
        RS485_ClearMeterRegisters();
        master_waiting = 0;
        energy_meter_master_state = 0;
        return;
    }
    expected_length = (uint8_t)(5 + meter_read_count * 2);
    if (port2_rx_buffer[1] != 0x03 ||
        port2_rx_buffer[2] != (meter_read_count * 2) ||
        length != expected_length)
    {
        energy_meter_last_error = ENERGY_METER_ERROR_BAD_RESPONSE;
        if (energy_meter_invalid_frame_count != 0xFFFF)
        {
            energy_meter_invalid_frame_count++;
        }
        energy_meter_connected = 0;
        energy_meter_data_valid = 0;
        RS485_ClearMeterRegisters();
        master_waiting = 0;
        energy_meter_master_state = 0;
        return;
    }
    for (index = 0; index < meter_read_count; index++)
    {
        energy_meter_registers[meter_read_offset + index] =
            ((uint16_t)port2_rx_buffer[3 + (index * 2)] << 8) |
            port2_rx_buffer[4 + (index * 2)];
        s_energy_meter_registers[meter_read_offset + index] =
            energy_meter_registers[meter_read_offset + index];
    }
    meter_retry_attempt = 0;
    meter_read_offset += meter_read_count;
    if (meter_read_offset < METER_REGISTER_COUNT)
    {
        master_waiting = 0;
        RS485_StartMeterRead();
        return;
    }
    meter_read_offset = 0;
    meter_retry_attempt = 0;
    energy_meter_connected = 1;
    energy_meter_data_valid = 1;
    EM_UpdateDecodedValues();
    /* The mapped timestamp assumes the BMS-synchronized RTC is in UTC. */
    pemc_last_poll_time = PEMC_RtcTimestamp();
    MeterEeprom_RequestSave();
    energy_meter_last_error = ENERGY_METER_ERROR_NONE;
    energy_meter_last_exception = 0;
    if (energy_meter_successful_responses != 0xFFFF)
    {
        energy_meter_successful_responses++;
    }
    master_waiting = 0;
    energy_meter_master_state = 0;
}

static uint16_t Config_Crc16(uint8_t xdata *buffer, uint8_t length)
{
    uint16_t crc;
    uint8_t index;
    uint8_t bit_index;

    crc = 0xFFFF;
    for (index = 0; index < length; index++)
    {
        crc ^= buffer[index];
        for (bit_index = 0; bit_index < 8; bit_index++)
            crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
    }
    return crc;
}

static void Config_PutU16(uint8_t xdata *record, uint8_t *cursor,
                          uint16_t value)
{
    record[(*cursor)++] = (uint8_t)(value >> 8);
    record[(*cursor)++] = (uint8_t)value;
}

static uint16_t Config_GetU16(uint8_t xdata *record, uint8_t *cursor)
{
    uint16_t value;
    value = ((uint16_t)record[(*cursor)++] << 8);
    value |= record[(*cursor)++];
    return value;
}

static void Config_PutU32(uint8_t xdata *record, uint8_t *cursor,
                          uint32_t value)
{
    record[(*cursor)++] = (uint8_t)(value >> 24);
    record[(*cursor)++] = (uint8_t)(value >> 16);
    record[(*cursor)++] = (uint8_t)(value >> 8);
    record[(*cursor)++] = (uint8_t)value;
}

static uint32_t Config_GetU32(uint8_t xdata *record, uint8_t *cursor)
{
    uint32_t value;
    value = (uint32_t)record[(*cursor)++] << 24;
    value |= (uint32_t)record[(*cursor)++] << 16;
    value |= (uint32_t)record[(*cursor)++] << 8;
    value |= record[(*cursor)++];
    return value;
}

static void Config_BuildRecord(void)
{
    uint8_t cursor;
    uint8_t index;
    uint16_t crc;

    meter_configuration_record[0] = 0x50;
    meter_configuration_record[1] = 0x4D;
    meter_configuration_record[2] = 1;
    meter_configuration_record[3] = CONFIG_RECORD_SIZE;
    cursor = 4;
    Config_PutU16(meter_configuration_record, &cursor, pemc_slave_id);
    meter_configuration_record[cursor++] = (uint8_t)pemc_parity;
    Config_PutU32(meter_configuration_record, &cursor, pemc_baud_rate);
    Config_PutU16(meter_configuration_record, &cursor, pemc_backlight_timeout);
    for (index = 0; index < 3; index++)
        Config_PutU32(meter_configuration_record, &cursor, pemc_unit_rates[index]);
    Config_PutU32(meter_configuration_record, &cursor, pemc_low_balance_threshold);
    meter_configuration_record[cursor++] = pemc_relay_command;
    meter_configuration_record[cursor++] = pemc_balance_source;
    for (index = 0; index < 10; index++)
        meter_configuration_record[cursor++] = pemc_customer_id[index];
    Config_PutU16(meter_configuration_record, &cursor, em_slave_id);
    meter_configuration_record[cursor++] = (uint8_t)em_parity;
    Config_PutU32(meter_configuration_record, &cursor, em_baud_rate);
    Config_PutU16(meter_configuration_record, &cursor, em_response_timeout);
    Config_PutU16(meter_configuration_record, &cursor, em_retry_count);
    Config_PutU16(meter_configuration_record, &cursor, em_poll_interval);
    for (index = 0; index < 6; index++)
    {
        Config_PutU16(meter_configuration_record, &cursor,
                      em_reading_addresses[index]);
        meter_configuration_record[cursor++] = em_reading_types[index];
        meter_configuration_record[cursor++] = em_reading_word_order[index];
        meter_configuration_record[cursor++] = (uint8_t)em_reading_scale[index];
    }
    crc = Config_Crc16(meter_configuration_record, CONFIG_RECORD_SIZE - 2);
    meter_configuration_record[CONFIG_RECORD_SIZE - 2] = (uint8_t)crc;
    meter_configuration_record[CONFIG_RECORD_SIZE - 1] = (uint8_t)(crc >> 8);
}

static void Config_LoadRecord(void)
{
    uint8_t cursor;
    uint8_t index;
    uint16_t stored_crc;

    if (!I2C_EEPROM_Read(CONFIG_EEPROM_OFFSET, meter_configuration_record,
                         CONFIG_RECORD_SIZE) ||
        meter_configuration_record[0] != 0x50 ||
        meter_configuration_record[1] != 0x4D ||
        meter_configuration_record[2] != 1 ||
        meter_configuration_record[3] != CONFIG_RECORD_SIZE)
        return;
    stored_crc = (uint16_t)(meter_configuration_record[CONFIG_RECORD_SIZE - 2] |
        ((uint16_t)meter_configuration_record[CONFIG_RECORD_SIZE - 1] << 8));
    if (stored_crc != Config_Crc16(meter_configuration_record,
                                   CONFIG_RECORD_SIZE - 2)) return;

    cursor = 4;
    pemc_slave_id = Config_GetU16(meter_configuration_record, &cursor);
    pemc_parity = meter_configuration_record[cursor++];
    pemc_baud_rate = Config_GetU32(meter_configuration_record, &cursor);
    pemc_backlight_timeout = Config_GetU16(meter_configuration_record, &cursor);
    for (index = 0; index < 3; index++)
        pemc_unit_rates[index] = Config_GetU32(meter_configuration_record, &cursor);
    pemc_low_balance_threshold = Config_GetU32(meter_configuration_record, &cursor);
    pemc_relay_command = meter_configuration_record[cursor++];
    pemc_balance_source = meter_configuration_record[cursor++];
    for (index = 0; index < 10; index++)
        pemc_customer_id[index] = meter_configuration_record[cursor++];
    em_slave_id = Config_GetU16(meter_configuration_record, &cursor);
    em_parity = meter_configuration_record[cursor++];
    em_baud_rate = Config_GetU32(meter_configuration_record, &cursor);
    em_response_timeout = Config_GetU16(meter_configuration_record, &cursor);
    em_retry_count = Config_GetU16(meter_configuration_record, &cursor);
    em_poll_interval = Config_GetU16(meter_configuration_record, &cursor);
    for (index = 0; index < 6; index++)
    {
        em_reading_addresses[index] = Config_GetU16(meter_configuration_record, &cursor);
        em_reading_types[index] = meter_configuration_record[cursor++];
        em_reading_word_order[index] = meter_configuration_record[cursor++];
        em_reading_scale[index] = (int8_t)meter_configuration_record[cursor++];
    }

    if (pemc_slave_id < 1 || pemc_slave_id > 247 || pemc_parity > 2 ||
        (pemc_baud_rate >> 16) != 0 || !RS485_BaudSupported((uint16_t)pemc_baud_rate) ||
        em_slave_id < 1 || em_slave_id > 247 || em_parity > 2 ||
        (em_baud_rate >> 16) != 0 || !RS485_BaudSupported((uint16_t)em_baud_rate) ||
        em_response_timeout == 0 || em_retry_count > 10 ||
        em_poll_interval == 0 || em_poll_interval > 60 ||
        pemc_relay_command > 1 || pemc_balance_source > 2)
    {
        pemc_slave_id = CONTROLLER_SLAVE_ID;
        pemc_parity = 1;
        pemc_baud_rate = METER_BAUD_RATE;
        pemc_backlight_timeout = 0;
        em_slave_id = METER_SLAVE_ID;
        em_parity = 1;
        em_baud_rate = METER_BAUD_RATE;
        em_response_timeout = MASTER_TIMEOUT_MS;
        em_retry_count = 0;
        em_poll_interval = 1;
        pemc_relay_command = 0;
        pemc_balance_source = 0;
        for (index = 0; index < 3; index++) pemc_unit_rates[index] = 0;
        pemc_low_balance_threshold = 0;
        for (index = 0; index < 10; index++) pemc_customer_id[index] = 0;
        for (index = 0; index < 6; index++)
        {
            em_reading_addresses[index] = 0xFFFF;
            em_reading_types[index] = 0;
            em_reading_word_order[index] = 0;
            em_reading_scale[index] = 0;
        }
    }
    else
    {
        for (index = 0; index < 6; index++)
        {
            if ((em_reading_addresses[index] != 0xFFFF &&
                 (em_reading_addresses[index] < METER_REGISTER_START ||
                  em_reading_addresses[index] >=
                    METER_REGISTER_START + METER_REGISTER_COUNT)) ||
                em_reading_types[index] > 2 || em_reading_word_order[index] > 1 ||
                em_reading_scale[index] < -5 || em_reading_scale[index] > 5)
            {
                em_reading_addresses[index] = 0xFFFF;
                em_reading_types[index] = 0;
                em_reading_word_order[index] = 0;
                em_reading_scale[index] = 0;
            }
        }
    }
}

static void Config_Service(void)
{
    uint8_t xdata verify[CONFIG_RECORD_SIZE];
    uint8_t index;

    if (!meter_configuration_dirty) return;
    if (meter_configuration_save_delay != 0)
    {
        meter_configuration_save_delay--;
        return;
    }
    Config_BuildRecord();
    if (!I2C_EEPROM_Write(CONFIG_EEPROM_OFFSET, meter_configuration_record,
                          CONFIG_RECORD_SIZE) ||
        !I2C_EEPROM_Read(CONFIG_EEPROM_OFFSET, verify, CONFIG_RECORD_SIZE))
    {
        meter_configuration_save_delay = 1000;
        return;
    }
    for (index = 0; index < CONFIG_RECORD_SIZE; index++)
    {
        if (verify[index] != meter_configuration_record[index])
        {
            meter_configuration_save_delay = 1000;
            return;
        }
    }
    meter_configuration_dirty = 0;
    meter_configuration_save_delay = 0;
}

uint16_t RS485_GetBacklightTimeout(void)
{
    return pemc_backlight_timeout;
}

void RS485_TestInit(void)
{
    uint8_t index;

    pemc_slave_id = CONTROLLER_SLAVE_ID;
    pemc_parity = 1;
    pemc_baud_rate = METER_BAUD_RATE;
    em_slave_id = METER_SLAVE_ID;
    em_parity = 1;
    em_baud_rate = METER_BAUD_RATE;
    em_response_timeout = MASTER_TIMEOUT_MS;
    em_retry_count = 0;
    em_poll_interval = 1;
    IE &= 0x7F;
    P37_INPUT_MODE;
    P36_QUASI_MODE;
    P06_PUSHPULL_MODE;
    P07_INPUT_MODE;
    ENABLE_UART1_TXD_P36;
    ENABLE_UART1_RXD_P37;
    SFRS = 0;
    for (index = 0; index < METER_REGISTER_COUNT; index++)
    {
        energy_meter_registers[index] = 0;
        s_energy_meter_registers[index] = 0;
#if ENERGY_METER_REGISTER_COUNT <= 4
        meter_eeprom_last_saved[index] = 0;
#endif
    }
    for (index = 0; index < 6; index++)
    {
        em_reading_addresses[index] = 0xFFFF;
        em_reading_types[index] = 0;
        em_reading_word_order[index] = 0;
        em_reading_scale[index] = 0;
        em_decoded_values[index] = 0;
    }
    em_decoded_values[6] = 0;
    for (index = 0; index < 3; index++) pemc_unit_rates[index] = 0;
    for (index = 0; index < 10; index++) pemc_customer_id[index] = 0;
    for (index = 0; index < 5; index++) pemc_tamper_times[index] = 0;
    pemc_low_balance_threshold = 0;
    pemc_relay_command = 0;
    pemc_balance_source = 0;
    pemc_relay_state = 0;
    pemc_low_balance_alert = 0;
    pemc_disconnect_reason = 0;
    pemc_tamper_flag = 0;
    pemc_last_poll_time = 0;
    pemc_backlight_timeout = 0;
    meter_configuration_dirty = 0;
    meter_configuration_save_delay = 0;
    Config_LoadRecord();
    pemc_active_slave_id = (uint8_t)pemc_slave_id;
    pemc_active_parity = pemc_parity;
    UART_Open(24000000UL, UART1_Timer3, pemc_baud_rate);
    UART_Open(24000000UL, UART0_Timer1, em_baud_rate);

    /* Mode 3 supplies the ninth bit for software parity or the 8N1 stop level. */
    SCON_1 = 0xD0;
    SCON = 0xD0;
    TI_1 = 0;
    TI = 0;
    RI_1 = 0;
    RI = 0;
    port1_tx_complete = 0;
    port2_tx_complete = 0;
    Board_SetRs485Transmit(1, 0);
    Board_SetRs485Transmit(2, 0);
    energy_meter_connected = 0;
    energy_meter_data_valid = 0;
    energy_meter_successful_responses = 0;
    energy_meter_master_state = 0;
    energy_meter_poll_count = 0;
    energy_meter_request_count = 0;
    energy_meter_last_error = ENERGY_METER_ERROR_NONE;
    energy_meter_timeout_count = 0;
    energy_meter_parity_error_count = 0;
    energy_meter_invalid_frame_count = 0;
    energy_meter_last_exception = 0;
    port1_rx_byte_count = 0;
    port1_parity_error_count = 0;
    port1_valid_request_count = 0;
    port1_response_count = 0;
    port1_bad_crc_count = 0;
    port1_wrong_address_count = 0;
    port1_last_frame_length = 0;
    for (index = 0; index < 8; index++)
    {
        port1_last_frame[index] = 0;
    }
    port1_rx_length = 0;
    port2_rx_length = 0;
    port1_silence_ms = 0;
    port2_silence_ms = 0;
    port1_rx_parity_error = 0;
    port2_rx_parity_error = 0;
    meter_eeprom_present = 0;
    meter_eeprom_loaded = 0;
    meter_eeprom_last_error = 0;
    meter_eeprom_save_count = 0;
    meter_eeprom_save_pending = 0;
    meter_eeprom_has_saved_record = 0;
    meter_eeprom_save_timer_ms = 0;
    MeterEeprom_Init();
#if ENERGY_METER_REGISTER_COUNT <= 4
    MeterEeprom_Load();
#endif
    meter_read_offset = 0;
    master_waiting = 0;
    master_timer_ms = (uint16_t)(em_poll_interval * 1000U);
    master_timeout_ms = 0;
    EA = 1;
}

void RS485_Poll(void)
{
    uint8_t frame_length;
    uint8_t parity_error;
    uint8_t global_interrupt_enabled;
    uint8_t index;

    SFRS = 0;
    if (energy_meter_poll_count != 0xFFFF)
    {
        energy_meter_poll_count++;
    }
    if (port2_rx_length != 0 && port2_silence_ms >= RTU_FRAME_GAP_MS)
    {
        global_interrupt_enabled = EA;
        EA = 0;
        frame_length = port2_rx_length;
        parity_error = port2_rx_parity_error;
        port2_rx_length = 0;
        port2_rx_parity_error = 0;
        EA = global_interrupt_enabled;
        if (!parity_error)
        {
            RS485_HandleMeterResponse(frame_length);
        }
        else
        {
            energy_meter_last_error = ENERGY_METER_ERROR_PARITY;
            if (energy_meter_parity_error_count != 0xFFFF)
            {
                energy_meter_parity_error_count++;
            }
        }
    }
    if (port1_rx_length != 0 && port1_silence_ms >= RTU_FRAME_GAP_MS)
    {
        global_interrupt_enabled = EA;
        EA = 0;
        frame_length = port1_rx_length;
        parity_error = port1_rx_parity_error;
        port1_rx_length = 0;
        port1_rx_parity_error = 0;
        port1_last_frame_length = frame_length;
        for (index = 0; index < frame_length && index < 8; index++)
        {
            port1_last_frame[index] = port1_rx_buffer[index];
        }
        EA = global_interrupt_enabled;
        if (!parity_error)
        {
            Modbus_HandleSlaveFrame(frame_length);
        }
    }

    if (master_waiting)
    {
        if (master_timeout_ms < em_response_timeout)
        {
            master_timeout_ms++;
        }
        else
        {
            if (meter_retry_attempt < em_retry_count)
            {
                meter_retry_attempt++;
                master_waiting = 0;
                port2_rx_length = 0;
                port2_rx_parity_error = 0;
                RS485_StartMeterRead();
            }
            else
            {
            master_waiting = 0;
            energy_meter_master_state = 0;
            port2_rx_length = 0;
            port2_rx_parity_error = 0;
            energy_meter_connected = 0;
            energy_meter_data_valid = 0;
            energy_meter_last_error = ENERGY_METER_ERROR_TIMEOUT;
            if (energy_meter_timeout_count != 0xFFFF)
            {
                energy_meter_timeout_count++;
            }
            RS485_ClearMeterRegisters();
            meter_read_offset = 0;
            meter_retry_attempt = 0;
            }
        }
    }
    else if (master_timer_ms < (uint16_t)(em_poll_interval * 1000U))
    {
        master_timer_ms++;
    }
    else
    {
        master_timer_ms = 0;
        RS485_StartMeterRead();
    }

    if (port1_silence_ms < RTU_FRAME_GAP_MS)
    {
        port1_silence_ms++;
    }
    if (port2_silence_ms < RTU_FRAME_GAP_MS)
    {
        port2_silence_ms++;
    }
    if (meter_eeprom_save_timer_ms < METER_EEPROM_SAVE_INTERVAL_MS)
    {
        meter_eeprom_save_timer_ms++;
    }
    Config_Service();
#if ENERGY_METER_REGISTER_COUNT <= 4
    MeterEeprom_Service();
#endif
}

