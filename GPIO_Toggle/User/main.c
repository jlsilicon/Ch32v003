/*
  Create project as : ch32v003 - f4p6  (- Not 103/203)
  - copy main.c from /EVT/EXAM/.../GPIO_Toggle/
*/

/*
 *@Note
 *GPIO routine:
 *PA0 push-pull output.
 */

#include "debug.h"

/*********************************************************************
 * @fn      GPIO_Toggle_INIT
 * @brief   Initializes GPIOD.4
 * @return  none
 */
void GPIO_Toggle_INIT(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOD, ENABLE );
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(         GPIOD,      &GPIO_InitStructure);
}

/*********************************************************************
 * @fn      main
 * @brief   Main program.
 * @return  none
 */
int main(void)
{

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    SystemCoreClockUpdate();
    Delay_Init();

    USART_Printf_Init(115200);
    printf("SystemClk:%d\r\n", SystemCoreClock);
    printf( "ChipID:%08x\r\n", DBGMCU_GetCHIPID() );
    printf("GPIO Toggle TEST\r\n");

    GPIO_Toggle_INIT();
    Delay_Ms(250);

    while(1)
    {
        printf("blink\r\n");

        GPIO_WriteBit(GPIOD, GPIO_Pin_4, Bit_RESET );
         Delay_Ms(250);

        GPIO_WriteBit(GPIOD, GPIO_Pin_4, Bit_SET );
         Delay_Ms(250);

    }

}

///
