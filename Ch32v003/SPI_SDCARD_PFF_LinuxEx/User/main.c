/*

// SD_CS_PIN   : PC0/PC3 : SD Card CS Push-Pull
// SD_SCK_PIN  : SCK on PC5, 10MHz Output, alt func, push-pull
// SD_MOSI_PIN : MOSI on PC6, 10MHz Output, alt func, push-pull
// SD_MISO_PIN : MISO on PC7, 10MHz input, floating

// #define IMAGE_FILENAME "testfil.txt"


Write Function :
The Petit FatFS module contains a write function with some restrictions, in order to minimize the required memory. 
These restrictions include:
* Files cannot be created; only an existing file can be written to
* Files cannot be expanded in size
* The time stamp of a file cannot be updated
* A write operation will only start and stop on a sector boundary
* A read-only file attribute will not block a write operation
This means that in order to write to a file, it must already exist on the SDC/MMC, and have the required
size allocated.


-WORKS-


Output:

: SPI SDCARD LINUX EX :
SystemClk:48000000
ChipID:00300500
Mounting volume.
Opening file "testfil.txt"
Loading image into RAM :

> SD Read : *TestFil.txt*
tetsing
123
Abc
Xyz
.
@p

Loaded 0 kilobytes.



*/

/*
 *@Note
 *Multiprocessor communication mode routine:
 *Master:USART1_Tx(PD5)\USART1_Rx(PD6).
 *This routine demonstrates that USART1 receives the data sent by CH341 and inverts
 *it and sends it (baud rate 115200).
 *
 *Hardware connection:PD5 -- Rx
 *                    PD6 -- Tx
 *
 */

#include "debug.h"

 #include "thing_config.h"


/* Global define */


/* Global Variable */
vu8 val;

/*********************************************************************
 * @fn      USARTx_CFG
 *
 * @brief   Initializes the USART2 & USART3 peripheral.
 *
 * @return  none
 */
void USARTx_CFG(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure = {0};
    USART_InitTypeDef USART_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_USART1, ENABLE);

    /* USART1 TX-->D.5   RX-->D.6 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;

    USART_Init(USART1, &USART_InitStructure);
    USART_Cmd(USART1, ENABLE);
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
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    SystemCoreClockUpdate();
    Delay_Init();

    USART_Printf_Init(115200);
     printf("\r\n\r\n: SPI SDCARD LINUX EX :\r\n" );
     printf("SystemClk:%d\r\n",SystemCoreClock);
     printf( "ChipID:%08x\r\n", DBGMCU_GetCHIPID() );

    USARTx_CFG();


    while(1)
    {

         main_2();

          Delay_Ms(1000);

    }


    while(1)
    {

        while(USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET)
        {
            /* waiting for receiving finish */
        }
        val = (USART_ReceiveData(USART1));
        USART_SendData(USART1, ~val);
        while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
        {
            /* waiting for sending finish */
        }
    }
}
