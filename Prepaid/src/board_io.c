#include "board_io.h"
#include "delay.h"

sbit RS485_1_DE_PIN = P3^3;
sbit RS485_1_DE_ALT_PIN = P2^5;
sbit RS485_2_DE_PIN = P0^1;

void Board_Init(void)
{
    SFRS = 0;

    RS485_1_DE_PIN = 0;
    RS485_1_DE_ALT_PIN = 0;
    RS485_2_DE_PIN = 0;

    P01_PUSHPULL_MODE;
    P25_PUSHPULL_MODE;
    P33_PUSHPULL_MODE;

    /* UART1 / RS-485 port 1 and UART0 / RS-485 port 2 pins. */
    P37_INPUT_MODE;
    P36_QUASI_MODE;
    P06_PUSHPULL_MODE;
    P07_INPUT_MODE;
}

void Board_DelayMs(uint16_t milliseconds)
{
    while (milliseconds >= 1000)
    {
        Timer0_Delay(24000000UL, 1000, 1000);
        milliseconds -= 1000;
    }
    if (milliseconds != 0)
    {
        Timer0_Delay(24000000UL, milliseconds, 1000);
    }
}

void Board_SetRs485Transmit(uint8_t port, uint8_t enable)
{
    SFRS = 0;
    if (port == 1)
    {
        RS485_1_DE_PIN = enable ? 1 : 0;
        RS485_1_DE_ALT_PIN = enable ? 1 : 0;
    }
    else if (port == 2)
    {
        RS485_2_DE_PIN = enable ? 1 : 0;
    }
}
