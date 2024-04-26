/*

 */

/*
 *@Note
 *Multiprocessor communication mode routine:
 *Master:USART1_Tx(PD5)\USART1_Rx(PD6).
 *This routine demonstrates that USART1 receives the data sent by CH341 and inverts
 *it and sends it (baud rate 115200).
 *
 *Hardware connection:PD5 -- Rx
 *                     PD6 -- Tx
 *
 */

#include "debug.h"

#include <ch32v00x_i2c.h>
#include <ch32v00x_rcc.h>


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

//

/*
 *@Note
 7-bit addressing mode, master/slave mode, transceiver routine:
 I2C1_SCL(PC2)\I2C1_SDA(PC1).
  This routine demonstrates that Master sends and Slave receives.
  Note: The two boards download the Master and Slave programs respectively,
   and power on at the same time.
      Hardware connection:
            PC2 -- PC2
            PC1 -- PC1

*/

#include "debug.h"

/* I2C Mode Definition */
#define HOST_MODE   0
#define SLAVE_MODE   1

/* I2C Communication Mode Selection */
#define I2C_MODE   HOST_MODE
//#define I2C_MODE   SLAVE_MODE

/* Global define */
#define Size   6
#define RXAdderss   0x02
#define TxAdderss   0x02

/* Global Variable */
u8 TxData[Size] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06 };
u8 RxData[5][Size];

/*********************************************************************
 * @fn      IIC_Init
 *
 * @brief   Initializes the IIC peripheral.
 *
 * @return  none
 */
void IIC_Init(u32 bound, u16 address)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    I2C_InitTypeDef  I2C_InitTSturcture = {0};

    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE );
    RCC_APB1PeriphClockCmd( RCC_APB1Periph_I2C1, ENABLE );

    /// SDA : ///
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOC, &GPIO_InitStructure );

    /// SCL : ///
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOC, &GPIO_InitStructure );

    I2C_InitTSturcture.I2C_Mode        = I2C_Mode_I2C;
    I2C_InitTSturcture.I2C_OwnAddress1 = address;
    I2C_InitTSturcture.I2C_ClockSpeed  = bound;
    I2C_InitTSturcture.I2C_DutyCycle   = I2C_DutyCycle_16_9;
    I2C_InitTSturcture.I2C_Ack         = I2C_Ack_Enable;
    I2C_InitTSturcture.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init( I2C1, &I2C_InitTSturcture );

    I2C_Cmd( I2C1, ENABLE );

// #if (I2C_MODE == HOST_MODE)
    I2C_AcknowledgeConfig( I2C1, ENABLE );

// #endif

}

//

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
     printf("\r\n\r\n: I2C_MPU6050_003_2 : \r\n");
     printf(" SystemClk:%d\r\n",SystemCoreClock);
     printf( "ChipID:%08x\r\n", DBGMCU_GetCHIPID() );

    USARTx_CFG();

    //

// u8  i = 0;
u8  j = 0;
// u8  p = 0;
u8  res ;

// __IO uint32_t  i2creg=0 ;
__IO uint32_t   i2cxbase=0 , i2cxbase1=0 , i2cxbase2=0 ;
  i2cxbase = (uint32_t)I2C1 ;  // = (uint32_t)I2Cx;
//  i2creg = I2C_FLAG >> 28;
//  I2C_FLAG &= FLAG_Mask;
//  if(i2creg != 0)
//  {
//    i2cxbase += 0x18;
    i2cxbase1 = i2cxbase + 0x14;
//  }
//  else
//  {
//    I2C_FLAG = (uint32_t)(I2C_FLAG >> 16);
//    i2cxbase += 0x18;
    i2cxbase2 = i2cxbase + 0x18;
//  }

//    Delay_Init();
//    USART_Printf_Init(460800);
//    printf("SystemClk:%d\r\n",SystemCoreClock);

printf("> main(): IIC mode\r\n");
    IIC_Init( 80000, TxAdderss);

  while( 1 )
  {

printf("= main(): for(0..5) \r\n");
    for( j=0x01; j <= 0x7F ; j++ )
    {
       while( I2C_GetFlagStatus( I2C1, I2C_FLAG_BUSY ) != RESET );

printf("= main(): ->I2C:Start() \r\n");
      I2C_GenerateSTART( I2C1, ENABLE );
       while( ! I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_MODE_SELECT ) ) {}

printf("  = main(): ->I2C:SendAdrId() = x%02X \r\n" , j );
//      I2C_Send7bitAddress( I2C1, 0x02, I2C_Direction_Transmitter );
//      I2C_Send7bitAddress( I2C1, 0x68, I2C_Direction_Transmitter );
      I2C_Send7bitAddress( I2C1, j, I2C_Direction_Transmitter );
//        while( ! I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED ) ) {}
      res = I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED ) ;
printf("    => MTM=%d " , res );
      res = I2C_CheckEvent( I2C1, I2C_Direction_Transmitter ) ;
printf(", DirT=%d " , res );
        Delay_Ms(1);

      res = I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED ) ;
printf(", MTM=%d " , res );
//      res = I2C_GetFlagStatus( I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED ) ;
      res = (*(__IO uint32_t *) i2cxbase1);
printf(", St=%d " , res );
      res = (*(__IO uint32_t *) i2cxbase2);
printf("%d " , res );

/*
//        while( I2C_GetFlagStatus( I2C1, I2C_FLAG_TXE ) ==  RESET ) {}
             Delay_Ms(100);
            I2C_SendData( I2C1, TxData[i] );
       while( ! I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED ) )  {}
*/

printf("\r\n= main(): ->I2C:Stop() \r\n");
      I2C_GenerateSTOP( I2C1, ENABLE );
//      res = I2C_GetFlagStatus( I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED ) ;
      res = (*(__IO uint32_t *) i2cxbase1);
printf("      , St=%d " , res );
      res = (*(__IO uint32_t *) i2cxbase2);
printf("%d \r\n" , res );

       Delay_Ms(100);
    }

printf("\r\n---\r\n\r\n");
    Delay_Ms(1000);
  }  // - while(1) .

    //

    while( 1 )
    {
        ;
    }

    //

/*
    while(1)
    {
        while(USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET)
        {
            // waiting for receiving finish //
        }
        val = (USART_ReceiveData(USART1));
        USART_SendData(USART1, ~val);
        while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
        {
            // waiting for sending finish //
        }
    }
*/
}

///
