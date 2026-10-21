#include "board_io.h"
#include "delay.h"

sbit RS485_1_DE_PIN = P3^3;
sbit RS485_1_DE_ALT_PIN = P2^5;
sbit RS485_2_DE_PIN = P0^1;
sbit BOARD_LED_1_PIN = P3^0;
sbit BOARD_LED_2_PIN = P1^7;
sbit RELAY_SET_1_PIN = P0^5;
sbit RELAY_SET_2_PIN = P0^4;
sbit RELAY_SET_3_PIN = P0^3;
sbit RELAY_RESET_1_PIN = P0^0;
sbit RELAY_RESET_2_PIN = P1^0;
sbit RELAY_RESET_3_PIN = P1^1;
sbit SHIFT_DATA_PIN = P2^1;
sbit SHIFT_CLOCK_PIN = P2^2;
sbit SHIFT_LATCH_PIN = P2^3;
sbit BOARD_SWITCH_1_PIN = P3^5;
sbit BOARD_SWITCH_2_PIN = P3^2;
sbit BOARD_SWITCH_3_PIN = P3^1;
sbit BOARD_BUZZER_PIN = P3^4;

#define SWITCH_DEBOUNCE_SAMPLES 20
#define SWITCH_1_LONG_PRESS_SAMPLES 5000

static uint8_t switch_debounce_counter[3];
static uint8_t switch_stable_pressed;
static uint16_t switch_1_hold_samples;
static uint8_t switch_1_long_press_reported;

void Board_Init(void)
{
    SFRS = 0;

    RS485_1_DE_PIN = 0;
    RS485_1_DE_ALT_PIN = 0;
    RS485_2_DE_PIN = 0;
    /* The board status LEDs are active-low, so high keeps them off. */
    BOARD_LED_1_PIN = 1;
    BOARD_LED_2_PIN = 1;
    RELAY_SET_1_PIN = 0;
    RELAY_SET_2_PIN = 0;
    RELAY_SET_3_PIN = 0;
    RELAY_RESET_1_PIN = 0;
    RELAY_RESET_2_PIN = 0;
    RELAY_RESET_3_PIN = 0;
    SHIFT_DATA_PIN = 0;
    SHIFT_CLOCK_PIN = 0;
    SHIFT_LATCH_PIN = 0;
    BOARD_BUZZER_PIN = 0;

    P01_PUSHPULL_MODE;
    P00_PUSHPULL_MODE;
    P03_PUSHPULL_MODE;
    P04_PUSHPULL_MODE;
    P05_PUSHPULL_MODE;
    P10_PUSHPULL_MODE;
    P11_PUSHPULL_MODE;
    P17_PUSHPULL_MODE;
    P30_PUSHPULL_MODE;
    P21_PUSHPULL_MODE;
    P22_PUSHPULL_MODE;
    P23_PUSHPULL_MODE;
    P35_INPUT_MODE;
    P32_INPUT_MODE;
    P31_INPUT_MODE;
    P34_PUSHPULL_MODE;
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

static void Board_RelayCoilsOff(void)
{
    SFRS = 0;
    RELAY_SET_1_PIN = 0;
    RELAY_SET_2_PIN = 0;
    RELAY_SET_3_PIN = 0;
    RELAY_RESET_1_PIN = 0;
    RELAY_RESET_2_PIN = 0;
    RELAY_RESET_3_PIN = 0;
}

void Board_PulseRelay(uint8_t channel, uint8_t turn_on, uint16_t milliseconds)
{
    SFRS = 0;
    if (channel == 1)
    {
        if (turn_on) RELAY_SET_1_PIN = 1;
        else RELAY_RESET_1_PIN = 1;
    }
    else if (channel == 2)
    {
        if (turn_on) RELAY_SET_2_PIN = 1;
        else RELAY_RESET_2_PIN = 1;
    }
    else if (channel == 3)
    {
        if (turn_on) RELAY_SET_3_PIN = 1;
        else RELAY_RESET_3_PIN = 1;
    }
    else
    {
        return;
    }

    Board_DelayMs(milliseconds);
    Board_RelayCoilsOff();
}

void Board_Shift595(uint8_t value)
{
    uint8_t bit_index;

    SFRS = 0;
    SHIFT_LATCH_PIN = 0;
    for (bit_index = 0; bit_index < 8; bit_index++)
    {
        SHIFT_CLOCK_PIN = 0;
        SHIFT_DATA_PIN = (value & 0x80) ? 1 : 0;
        SHIFT_CLOCK_PIN = 1;
        value <<= 1;
    }
    SHIFT_CLOCK_PIN = 0;
    SHIFT_LATCH_PIN = 1;
    SHIFT_LATCH_PIN = 0;
}

uint8_t Board_ReadSwitchPresses(void)
{
    uint8_t raw_pressed;
    uint8_t button_mask;
    uint8_t button_index;
    uint8_t pressed;
    uint8_t events;

    SFRS = 0;
    raw_pressed = 0;
    if (!BOARD_SWITCH_1_PIN) raw_pressed |= BOARD_SWITCH_1;
    if (!BOARD_SWITCH_2_PIN) raw_pressed |= BOARD_SWITCH_2;
    if (!BOARD_SWITCH_3_PIN) raw_pressed |= BOARD_SWITCH_3;

    events = 0;
    button_mask = BOARD_SWITCH_1;
    for (button_index = 0; button_index < 3; button_index++)
    {
        pressed = (raw_pressed & button_mask) ? 1 : 0;
        if (pressed != ((switch_stable_pressed & button_mask) ? 1 : 0))
        {
            if (switch_debounce_counter[button_index] < SWITCH_DEBOUNCE_SAMPLES)
            {
                switch_debounce_counter[button_index]++;
            }
            if (switch_debounce_counter[button_index] >= SWITCH_DEBOUNCE_SAMPLES)
            {
                switch_debounce_counter[button_index] = 0;
                if (pressed)
                {
                    switch_stable_pressed |= button_mask;
                    events |= button_mask;
                }
                else
                {
                    switch_stable_pressed &= (uint8_t)~button_mask;
                }
            }
        }
        else
        {
            switch_debounce_counter[button_index] = 0;
        }
        button_mask <<= 1;
    }

    if (switch_stable_pressed & BOARD_SWITCH_1)
    {
        if (switch_1_hold_samples < SWITCH_1_LONG_PRESS_SAMPLES)
        {
            switch_1_hold_samples++;
        }
        if (switch_1_hold_samples >= SWITCH_1_LONG_PRESS_SAMPLES &&
            !switch_1_long_press_reported)
        {
            events |= BOARD_SWITCH_1_LONG;
            switch_1_long_press_reported = 1;
        }
    }
    else
    {
        switch_1_hold_samples = 0;
        switch_1_long_press_reported = 0;
    }
    return events;
}

void Board_Beep(uint16_t milliseconds)
{
    SFRS = 0;
    BOARD_BUZZER_PIN = 1;
    Board_DelayMs(milliseconds);
    BOARD_BUZZER_PIN = 0;
}
