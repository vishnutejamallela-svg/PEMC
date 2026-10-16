#include "lcd_595.h"
#include "board_io.h"

#define LCD_RS_MASK 0x20
#define LCD_EN_MASK 0x40
#define LCD_BACKLIGHT_MASK 0x01

static void LCD_WriteNibble(uint8_t nibble, uint8_t is_data)
{
    uint8_t value;

    value = (uint8_t)((nibble & 0x0F) << 1);
    value |= LCD_BACKLIGHT_MASK;
    if (is_data) value |= LCD_RS_MASK;
    Board_Shift595((uint8_t)(value | LCD_EN_MASK));
    Board_Shift595(value);
}

static void LCD_WriteByte(uint8_t value, uint8_t is_data)
{
    LCD_WriteNibble((uint8_t)(value >> 4), is_data);
    LCD_WriteNibble(value, is_data);
}

void LCD_Command(uint8_t command)
{
    LCD_WriteByte(command, 0);
    if (command == 0x01 || command == 0x02)
    {
        Board_DelayMs(2);
    }
}

void LCD_Init(void)
{
    Board_DelayMs(40);
    LCD_WriteNibble(0x03, 0);
    Board_DelayMs(5);
    LCD_WriteNibble(0x03, 0);
    Board_DelayMs(1);
    LCD_WriteNibble(0x03, 0);
    LCD_WriteNibble(0x02, 0);
    LCD_Command(0x28);
    LCD_Command(0x0C);
    LCD_Command(0x06);
    LCD_Command(0x01);
}

void LCD_WriteChar(char value)
{
    LCD_WriteByte((uint8_t)value, 1);
}

void LCD_WriteText(char code *text)
{
    while (*text != '\0')
    {
        LCD_WriteChar(*text++);
    }
}

void LCD_SetCursor(uint8_t row, uint8_t column)
{
    uint8_t address;

    address = (row == 0) ? 0x00 : 0x40;
    LCD_Command((uint8_t)(0x80 | (address + column)));
}

void LCD_WriteHex(uint8_t value)
{
    uint8_t digit;

    digit = (uint8_t)(value >> 4);
    LCD_WriteChar((char)(digit < 10 ? ('0' + digit) : ('A' + digit - 10)));
    digit = (uint8_t)(value & 0x0F);
    LCD_WriteChar((char)(digit < 10 ? ('0' + digit) : ('A' + digit - 10)));
}
