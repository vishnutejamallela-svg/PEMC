#include "interface_test.h"
#include "board_io.h"
#include "i2c_probe.h"
#include "lcd_595.h"
#include "rs485_test.h"

static I2C_TestResult i2c_result;
static RS485_TestResult rs485_result;
static uint8_t previous_key_3;
static uint8_t screen_page;
static uint16_t screen_timer;

static void InterfaceTest_ShowStatus(void)
{
    uint16_t meter_value;

    LCD_SetCursor(0, 0);
    LCD_WriteText("40001:");
    if (RS485_IsMeterDataValid())
    {
        meter_value = RS485_GetMeterRegister(0);
        LCD_WriteHex((uint8_t)(meter_value >> 8));
        LCD_WriteHex((uint8_t)meter_value);
        LCD_WriteText(" OK ");
    }
    else
    {
        LCD_WriteText("---- WAIT");
    }

    LCD_SetCursor(1, 0);
    LCD_WriteText("RLY1=");
    LCD_WriteChar(RS485_GetRelayState(1) ? '1' : '0');
    LCD_WriteText(" 2=");
    LCD_WriteChar(RS485_GetRelayState(2) ? '1' : '0');
    LCD_WriteText(" 3=");
    LCD_WriteChar(RS485_GetRelayState(3) ? '1' : '0');
}

void InterfaceTest_Init(void)
{
    previous_key_3 = 0;
    screen_page = 0;
    screen_timer = 0;
    rs485_result.port1_rx_count = 0;
    rs485_result.port2_rx_count = 0;
    rs485_result.last_port = 0;
    rs485_result.last_byte = 0;

    LCD_SetCursor(0, 0);
    LCD_WriteText("PREPAID IF TEST ");
    LCD_SetCursor(1, 0);
    LCD_WriteText("MS51PC0AE        ");

    I2C_RunReadOnlyTests(&i2c_result);

    LCD_Command(0x01);
    InterfaceTest_ShowStatus();
}

void InterfaceTest_Poll(void)
{
    uint8_t key_3;

    RS485_Poll(&rs485_result);
    Board_SetLed(1, RS485_IsMeterDataValid());
    Board_SetLed(2, RS485_IsMeterDataValid());
    key_3 = Board_KeyPressed(3);

    if (key_3 && !previous_key_3)
    {
        screen_page ^= 1;
    }

    previous_key_3 = key_3;

    if (screen_timer >= 500)
    {
        screen_timer = 0;
        if (screen_page == 0)
        {
            InterfaceTest_ShowStatus();
        }
        else
        {
            LCD_SetCursor(0, 0);
            LCD_WriteText("RTC:");
            LCD_WriteChar(i2c_result.rtc_read_ok ? 'Y' : 'N');
            LCD_WriteText(" EE:");
            LCD_WriteChar(i2c_result.eeprom_read_ok ? 'Y' : 'N');
            LCD_SetCursor(1, 0);
            LCD_WriteText("IR:");
            LCD_WriteChar(Board_IrActive() ? '1' : '0');
            LCD_WriteText(" RS2:MASTER");
        }
    }

    Board_DelayMs(1);
    screen_timer++;
}
