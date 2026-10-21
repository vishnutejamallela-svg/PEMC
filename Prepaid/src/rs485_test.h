#ifndef RS485_TEST_H
#define RS485_TEST_H

#include "ms51_device.h"

#define ENERGY_METER_REGISTER_COUNT 200

/*
 * PEMC Modbus map uses zero-based PDU offsets (ModScan holding reference is
 * 40001 + offset): PEMC 40001..40026, EM settings 40051..40101,
 * EM source addresses 40102..40151, application values 40201..40301,
 * and the raw 200-register EM window 40302..40501.
 */
#define PEMC_SETTINGS_START          0U
#define PEMC_SETTINGS_END            25U
#define ENERGY_METER_SETTINGS_START  50U
#define ENERGY_METER_SETTINGS_END   100U
#define ENERGY_METER_ADDRESS_START  101U
#define ENERGY_METER_ADDRESS_END    150U
#define APPLICATION_VALUES_START    200U
#define APPLICATION_VALUES_END      300U
#define RAW_ENERGY_METER_START      301U
#define SLAVE_HOLDING_REGISTER_COUNT \
    (RAW_ENERGY_METER_START + ENERGY_METER_REGISTER_COUNT)

/* Raw meter cache appears at ModScan 40302..40501. */
#define RAW_ENERGY_METER_FIRST_REFERENCE (40001U + RAW_ENERGY_METER_START)

/* Add these globals to Keil Watch 1 while the target is running. */
extern volatile uint8_t energy_meter_connected;
extern volatile uint8_t energy_meter_data_valid;
extern volatile uint16_t energy_meter_successful_responses;
extern volatile uint8_t energy_meter_master_state;
extern volatile uint16_t energy_meter_poll_count;
extern volatile uint16_t energy_meter_request_count;
extern volatile uint8_t energy_meter_last_error;
extern volatile uint16_t energy_meter_timeout_count;
extern volatile uint16_t energy_meter_parity_error_count;
extern volatile uint16_t energy_meter_invalid_frame_count;
extern volatile uint8_t energy_meter_last_exception;
extern volatile uint16_t port1_rx_byte_count;
extern volatile uint16_t port1_parity_error_count;
extern volatile uint16_t port1_valid_request_count;
extern volatile uint16_t port1_response_count;
extern volatile uint16_t port1_bad_crc_count;
extern volatile uint16_t port1_wrong_address_count;
extern volatile uint8_t port1_last_frame_length;
extern volatile uint8_t xdata port1_last_frame[8];
extern volatile uint8_t meter_eeprom_present;
extern volatile uint8_t meter_eeprom_loaded;
extern volatile uint8_t meter_eeprom_last_error;
extern volatile uint16_t meter_eeprom_save_count;
extern volatile uint16_t xdata energy_meter_registers[ENERGY_METER_REGISTER_COUNT];
extern volatile uint16_t xdata s_energy_meter_registers[ENERGY_METER_REGISTER_COUNT];

/* energy_meter_last_error values */
#define ENERGY_METER_ERROR_NONE          0
#define ENERGY_METER_ERROR_TIMEOUT       1
#define ENERGY_METER_ERROR_PARITY        2
#define ENERGY_METER_ERROR_BAD_FRAME     3
#define ENERGY_METER_ERROR_EXCEPTION     4
#define ENERGY_METER_ERROR_BAD_RESPONSE  5

void RS485_TestInit(void);
void RS485_Poll(void);
uint16_t RS485_GetBacklightTimeout(void);

#endif
