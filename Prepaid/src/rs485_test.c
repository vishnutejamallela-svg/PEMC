#include "rs485_test.h"
#include "board_io.h"
#include "uart.h"

#define METER_SLAVE_ID          4
#define CONTROLLER_SLAVE_ID     1
#define METER_BAUD_RATE         9600UL
#define METER_REGISTER_START    0
#define METER_REGISTER_COUNT    ENERGY_METER_REGISTER_COUNT
#define SLAVE_REGISTER_COUNT    METER_REGISTER_COUNT
#define RTU_FRAME_GAP_MS        4
#define MASTER_TIMEOUT_MS       300
#define MASTER_POLL_INTERVAL_MS 1000
#define RX_BUFFER_SIZE          48
#define METER_EEPROM_ADDRESS    0x50
#define METER_EEPROM_OFFSET     0x0020
#define METER_EEPROM_RECORD_SIZE 14
#define METER_EEPROM_SAVE_INTERVAL_MS 60000UL

#define I2C_SI                  0x08
#define I2C_STO                 0x10
#define I2C_STA                 0x20
#define I2C_ENABLE              0x40

static uint8_t xdata port1_rx_buffer[RX_BUFFER_SIZE];
static uint8_t xdata port2_rx_buffer[RX_BUFFER_SIZE];
static uint8_t xdata tx_buffer[RX_BUFFER_SIZE];
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
static uint16_t xdata meter_eeprom_last_saved[METER_REGISTER_COUNT];
static uint8_t xdata meter_eeprom_record[METER_EEPROM_RECORD_SIZE];
static uint8_t meter_eeprom_save_pending;
static uint8_t meter_eeprom_has_saved_record;
static uint32_t meter_eeprom_save_timer_ms;

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

static void RS485_SendByte(uint8_t port, uint8_t value)
{
    uint8_t parity;

    parity = RS485_EvenParity(value);
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
    uint8_t bit_index;
    uint16_t crc;

    crc = 0xFFFF;
    for (index = 0; index < length; index++)
    {
        crc ^= buffer[index];
        for (bit_index = 0; bit_index < 8; bit_index++)
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

static void MeterEeprom_RequestSave(void)
{
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
}

static void MeterEeprom_Service(void)
{
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
        if (RB8 != RS485_EvenParity(value))
        {
            port2_rx_parity_error = 1;
        }
        if (port2_rx_length < RX_BUFFER_SIZE)
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
        if (port1_rx_byte_count != 0xFFFF)
        {
            port1_rx_byte_count++;
        }
        if (RB8_1 != RS485_EvenParity(value))
        {
            port1_rx_parity_error = 1;
            if (port1_parity_error_count != 0xFFFF)
            {
                port1_parity_error_count++;
            }
        }
        if (port1_rx_length < RX_BUFFER_SIZE)
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
    if (port == 1)
    {
        Board_DelayMs(1);
    }
    for (index = 0; index < (uint8_t)(length + 2); index++)
    {
        RS485_SendByte(port, frame[index]);
    }
    Board_DelayMs(2);
    Board_SetRs485Transmit(port, 0);
}

static void Modbus_SendException(uint8_t function, uint8_t exception_code)
{
    tx_buffer[0] = CONTROLLER_SLAVE_ID;
    tx_buffer[1] = function | 0x80;
    tx_buffer[2] = exception_code;
    RS485_SendFrame(1, tx_buffer, 3);
}

static void Modbus_HandleSlaveFrame(uint8_t length)
{
    uint8_t function;
    uint8_t quantity;
    uint8_t index;
    uint16_t start_address;
    uint16_t value;

    if (!Modbus_FrameValid(port1_rx_buffer, length))
    {
        if (port1_bad_crc_count != 0xFFFF)
        {
            port1_bad_crc_count++;
        }
        return;
    }
    if (port1_rx_buffer[0] != CONTROLLER_SLAVE_ID)
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

    if (function != 0x03)
    {
        Modbus_SendException(function, 0x01);
        return;
    }
    if (length != 8)
    {
        return;
    }
    if (port1_rx_buffer[4] != 0)
    {
        Modbus_SendException(function, 0x03);
        return;
    }
    quantity = port1_rx_buffer[5];
    if (quantity == 0 || quantity > SLAVE_REGISTER_COUNT ||
        start_address >= SLAVE_REGISTER_COUNT ||
        (uint16_t)(start_address + quantity) > SLAVE_REGISTER_COUNT)
    {
        Modbus_SendException(function, 0x02);
        return;
    }

    tx_buffer[0] = CONTROLLER_SLAVE_ID;
    tx_buffer[1] = function;
    tx_buffer[2] = quantity * 2;
    for (index = 0; index < quantity; index++)
    {
        value = s_energy_meter_registers[start_address + index];
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
    tx_buffer[0] = METER_SLAVE_ID;
    tx_buffer[1] = 0x03;
    tx_buffer[2] = (uint8_t)(METER_REGISTER_START >> 8);
    tx_buffer[3] = (uint8_t)METER_REGISTER_START;
    tx_buffer[4] = 0;
    tx_buffer[5] = METER_REGISTER_COUNT;
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
        port2_rx_buffer[0] != METER_SLAVE_ID)
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
    expected_length = (uint8_t)(5 + METER_REGISTER_COUNT * 2);
    if (port2_rx_buffer[1] != 0x03 ||
        port2_rx_buffer[2] != (METER_REGISTER_COUNT * 2) ||
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
    for (index = 0; index < METER_REGISTER_COUNT; index++)
    {
        energy_meter_registers[index] =
            ((uint16_t)port2_rx_buffer[3 + (index * 2)] << 8) |
            port2_rx_buffer[4 + (index * 2)];
			
        s_energy_meter_registers[index] = energy_meter_registers[index];
    }
    energy_meter_connected = 1;
    energy_meter_data_valid = 1;
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

void RS485_TestInit(void)
{
    uint8_t index;

    IE &= 0x7F;
    P37_INPUT_MODE;
    P36_QUASI_MODE;
    P06_PUSHPULL_MODE;
    P07_INPUT_MODE;
    ENABLE_UART1_TXD_P36;
    ENABLE_UART1_RXD_P37;
    SFRS = 0;
    UART_Open(24000000UL, UART1_Timer3, METER_BAUD_RATE);
    UART_Open(24000000UL, UART0_Timer1, METER_BAUD_RATE);

    /* Mode 3 supplies the ninth bit used here as manually generated even parity. */
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
    for (index = 0; index < METER_REGISTER_COUNT; index++)
    {
        energy_meter_registers[index] = 0;
        s_energy_meter_registers[index] = 0;
        meter_eeprom_last_saved[index] = 0;
    }
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
    MeterEeprom_Load();
    master_waiting = 0;
    master_timer_ms = MASTER_POLL_INTERVAL_MS;
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
        if (master_timeout_ms < MASTER_TIMEOUT_MS)
        {
            master_timeout_ms++;
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
        }
    }
    else if (master_timer_ms < MASTER_POLL_INTERVAL_MS)
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
    MeterEeprom_Service();
}

