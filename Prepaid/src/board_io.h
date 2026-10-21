#ifndef BOARD_IO_H
#define BOARD_IO_H

#include "ms51_device.h"

#define BOARD_SWITCH_1 0x01
#define BOARD_SWITCH_2 0x02
#define BOARD_SWITCH_3 0x04
#define BOARD_SWITCH_1_LONG 0x08

void Board_Init(void);
void Board_DelayMs(uint16_t milliseconds);
void Board_SetRs485Transmit(uint8_t port, uint8_t enable);
void Board_PulseRelay(uint8_t channel, uint8_t turn_on, uint16_t milliseconds);
void Board_Shift595(uint8_t value);
uint8_t Board_ReadSwitchPresses(void);
void Board_Beep(uint16_t milliseconds);

#endif
