#ifndef BOARD_IO_H
#define BOARD_IO_H

#include "ms51_device.h"

void Board_Init(void);
void Board_DelayMs(uint16_t milliseconds);
void Board_SetRs485Transmit(uint8_t port, uint8_t enable);

#endif
