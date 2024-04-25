/*
 FAQs :
 - Ch32v203c8t6 did Not Flash because short across Cable Pins

 - LED= PB2

 - TX = PA9
 - RX = PA10

 - WORKS
*/

/*
 *@Note
 *GPIO routine:
 *PA0 push-pull output.
 *
 */

#include "debug.h"

/* Global define */

/* Global Variable */

/*********************************************************************
 * @fn      GPIO_Toggle_INIT
 *
 * @brief   Initializes GPIOA.0
 *
 * @return  none
 */
void GPIO_Toggle_INIT(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};

/*
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
*/

    // LED = PB2 on Ch32v203c8t6 : //
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

}

/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
int main(void)
{
    u8 i = 0;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    SystemCoreClockUpdate();
    Delay_Init();

    USART_Printf_Init(115200);
     printf("SystemClk:%d\r\n", SystemCoreClock);
     printf("ChipID:%08x\r\n" , DBGMCU_GetCHIPID() );
     printf("GPIO Toggle TEST\r\n");

    GPIO_Toggle_INIT();

    while(1)
    {
        printf("blink...\r\n");

//        GPIO_WriteBit(GPIOA, GPIO_Pin_0, (i == 0) ? (i = Bit_SET) : (i = Bit_RESET));

        // LED on Ch32v203c8t6 : //
        GPIO_WriteBit(GPIOB, GPIO_Pin_2, (i == 0) ? (i = Bit_SET) : (i = Bit_RESET));

        Delay_Ms(250);

    }
}

///
