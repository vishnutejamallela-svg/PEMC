#include "ms51_device.h"
#include "sys.h"
#include "board_io.h"
#include "rs485_test.h"

void main(void)
{
    MODIFY_HIRC(HIRC_24);
    SFRS = 0;
    Board_Init();
    RS485_TestInit();

    while (1)
    {
        RS485_Poll();
        Board_DelayMs(1);
    }
}
