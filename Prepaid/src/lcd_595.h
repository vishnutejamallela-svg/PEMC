#ifndef LCD_595_H
#define LCD_595_H

#include "ms51_device.h"

void LCD_Init(void);
void LCD_Command(uint8_t command);
void LCD_WriteChar(char value);
void LCD_WriteText(char code *text);
void LCD_SetCursor(uint8_t row, uint8_t column);
void LCD_WriteHex(uint8_t value);

#endif
