#ifndef LCD_595_H
#define LCD_595_H

#include "ms51_device.h"

#define BACK_LIGHT_OFF 0
#define BACK_LIGHT_ON  1

extern volatile uint8_t back_light_on;

/* Keep existing source/debug references working after the clearer name. */
#define backlight back_light_on

void LCD_Init(void);
void LCD_ApplyBacklight(void);
void LCD_Command(uint8_t command);
void LCD_WriteChar(char value);
void LCD_WriteText(char code *text);
void LCD_SetCursor(uint8_t row, uint8_t column);
void LCD_WriteHex(uint8_t value);

#endif
