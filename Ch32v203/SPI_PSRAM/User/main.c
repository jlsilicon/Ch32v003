/*
 FAQs :
 - need Pullup on NSS pin
 - need SPI_InitStructure.SPI_FirstBit = should be SPI_FirstBit_MSB , Not SPI_FirstBit_LSB;
 - need SPI_InitStructure.SPI_DataSize = should be SPI_DataSize_18b , Not SPI_DataSize_16b;
 - need SPI_InitStructure.SPI_NSS      = should be SPI_NSS_Soft , Not SPI_NSS_Hard ;

 Compiles,Downloads,Run,Output:

: SPI_PSRM_203 :
- SystemClk:96000000
= main() : -> SPI_FullDuplex_Init()
= main() : Host Mode
= main() : copying Str_=*Psram* : ABCdefghijklmnopqrstunWXYZ_0123456789_***
 = main() : SPI NSS=->H

= main() : Adr_=0
= main() : SPI Toggle NSS=PA4=->H->L
= main() : SPI send Cmd RESE=0x66
= main() : SPI Toggle NSS=PA4=->H->L
= main() : SPI send Cmd RES=0x99
= main() : SPI NSS=->H

= main() : SPI Toggle NSS=PA4=->H->L
= main() : SPI send Cmd WRT=0x02 : , Adr HIGH=00  , Adr MID=00  , Adr LOW=00
= main() : SPI send Str_=*Psram* : ABCdefghijklmnopqrstunWXYZ_0123456789_***
= main() : SPI NSS=->H

= main() : SPI Toggle PA4=->H->L
= main() : SPI send Cmd RD=0x03 : , Adr HIGH=00  , Adr MID=00  , Adr LOW=00
= main() : SPI read BYTE
 <=0 0<  <=0 0<  <=0 0<  <=0 0<  <=0 0<  <=0 0<  <=0 0<  <=0 0<
 <=0 0<  <=0 0<  <=0 0<  <=0 0<  <=0 0<  <=0 0<  <=0 0<  <=0 0<
 <=0 0<  <=0 0<  <=0 0<  <=0 0<  <=0 0<  <=0 0<  <=0 0<  <=0 0<
 <=0 0<  <=0 0<  <=0 0<  <=0 0<  <=0 0<  <=0 0<  <=0 0<  <=0 0<
 <=0 0<  <=0 0<  <=0 0<  <=0 0<  <=0 0<  <=0 0<  <=0 0<  <=0 0<
 <=0 0<  <=0 0<  <=0 0<  <=0 0<  <=0 0<  <=0 0<  <=0 0<  <=0 0<
 <=0 0<  <=0 0<  <=0 0<  <=0 0<  <=0 0<  <=0 0<  <=0 0<  <=0 0<
 <=0 0<  <=0 0<  <=0 0<  <=0 0<  <=0 0<  <=0 0<  <=0 0<  <=0 0<

= main() : SPI Toggle SS=PC1,PA4=->H->L
= main() : SPI send Cmd RD=0x03 :  , Adr HIGH=00  , Adr MID=00  , Adr LOW=00

 <=0 0<  <=0 0<  <=0 0<  <=0 0<  <=0 0<  <=0 0<  <=0 0<  <=0 0<
 <=0 0<  <=0 0<  <=0 0<  <=0 0<  <=0 0<  <=0 0<  <=0 0<  <=0 0<
 <=0 0<  <=0 0<  <=0 0<  <=0 0<  <=0 0<  <=0 0<  <=0 0<  <=0 0<
 <=0 0<  <=0 0<  <=0 0<  <=0 0<  <=0 0<  <=0 0<  <=0 0<  <=0 0<
 <=0 0<  <=0 0<  <=0 0<  <=0 0<  <=0 0<  <=0 0<  <=0 0<  <=0 0<
 <=0 0<  <=0 0<  <=0 0<  <=0 0<  <=0 0<  <=0 0<  <=0 0<  <=0 0<
 <=0 0<  <=0 0<  <=0 0<  <=0 0<  <=0 0<  <=0 0<  <=0 0<  <=0 0<
 <=0 0<  <=0 0<  <=0 0<  <=0 0<  <=0 0<  <=0 0<  <=0 0<  <=0 0<

= main() : SPI Toggle SS=PC1,PA4=->H->L

*/

/*
 *@Note
 Two-wire full duplex mode, master/slave mode, data transceiver:
 *Master : SPI1_NSS(PA4) , SPI1_SCK(PA5) , SPI1_MISO(PA6) , SPI1_MOSI(PA7).
 *Slave  : SPI1_NSS(PA4) , SPI1_SCK(PA5) , SPI1_MISO(PA6) , SPI1_MOSI(PA7).
 *
 *This example demonstrates that in hardware NSS mode, the Master and Slave can
 *transmit and receive in full duplex at the same time.
 *Note: The two boards download the Master and Slave programs respectively, and
 *power on at the same time.It is recommended that the NSS pin be connected to
 *a 10K pull-up resistor.
 *    Hardware connection:
 *                PA4 -- PA4 = NSS
 *                PA5 -- PA5 = SCK
 *                PA6 -- PA6 = MISO
 *                PA7 -- PA7 = MOSI


*/


#include "debug.h"
#include "string.h"

//

/* SPI Mode Definition */
#define HOST_MODE     0
#define SLAVE_MODE    1

/* SPI Communication Mode Selection */
#define SPI_MODE      HOST_MODE
// #define SPI_MODE   SLAVE_MODE

/* Global define */

/* Global Variable */

// #define Size          255
// u16 TxData[Size] ;
// u16 RxData[Size] ;


/*********************************************************************
 * @fn      SPI_FullDuplex_Init
 *
 * @brief   Configuring the SPI for full-duplex communication.
 *
 * @return  none
 */
void SPI_FullDuplex_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    SPI_InitTypeDef  SPI_InitStructure  = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_SPI1, ENABLE);

#if(SPI_MODE == HOST_MODE)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;  // PD4
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;  // PC5
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6; // PC6
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;  // PC7
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

#elif(SPI_MODE == SLAVE_MODE)
/*
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
*/
#endif

#if(SPI_MODE == HOST_MODE)
    SPI_SSOutputCmd(SPI1, ENABLE);

#endif

    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;

#if(SPI_MODE == HOST_MODE)
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;

#elif(SPI_MODE == SLAVE_MODE)
    SPI_InitStructure.SPI_Mode = SPI_Mode_Slave;

#endif

//    SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL     = SPI_CPOL_Low;
    SPI_InitStructure.SPI_CPHA     = SPI_CPHA_1Edge;
//    SPI_InitStructure.SPI_NSS      = SPI_NSS_Hard;
    SPI_InitStructure.SPI_NSS      = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;
//    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_LSB;
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
//    SPI_InitStructure.SPI_CRCPolynomial = 7;
    SPI_Init(SPI1, &SPI_InitStructure);

    SPI_Cmd(SPI1, ENABLE);
}


char  Str_[ 128 ] ;
int   Str_L ;
// char  Str_ = "*Psram* : ABCdefghijklmnopqrstunWXYZ_0123456789_***\r\n" ;
// int   Str_L = strlen( Str_ );


/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
int main(void)
{
// u8  i = 0;
// u8  j = 0;
// u8  value;
u8  RxData_1 ;
u8  Adr_ = 0 ;


    Delay_Init();
    USART_Printf_Init(115200);
     printf("\r\n\r\n: SPI_PSRM_203 :\r\n");
     printf("- SystemClk:%d\r\n", SystemCoreClock);

#if(SPI_MODE == SLAVE_MODE)
/*
    printf("Slave Mode\r\n");
    Delay_Ms(1000);
*/
#endif

printf("= main() : -> SPI_FullDuplex_Init() \r\n" , Str_ );
    SPI_FullDuplex_Init();

#if(SPI_MODE == HOST_MODE)
printf("= main() : Host Mode\r\n");
    Delay_Ms(1000);

#endif

    //

    strcpy( Str_ , "*Psram* : ABCdefghijklmnopqrstunWXYZ_0123456789_***\r\n" );
    Str_L = strlen( Str_ );
//printf("= main() : copying Str_=%s \r\n" , Str_ );
printf("= main() : copying Str_=%s " , Str_ );

    /// SPI NSS Set ->H = DISable : ///
printf("= main() : SPI NSS=->H \r\n\r\n" , Str_ );
    SPI_SSOutputCmd(SPI1, DISABLE);
     Delay_Ms(1);


printf("= main() : Adr_=0 \r\n" , Str_ );
    Adr_ = 0 ;


    //

printf("= main() : SPI Toggle NSS=PA4=->H->L \r\n" , Str_ );
    /// SPI NSS Toggle ->H->L : ///
    SPI_SSOutputCmd(SPI1, DISABLE);
     Delay_Ms(1);
    SPI_SSOutputCmd(SPI1, ENABLE);

printf("= main() : SPI send Cmd RESE=0x66 \r\n" , Str_ );
    /// SPI send Cmd RESE=0x66 : ///
     while( SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET )  {}
    SPI_I2S_SendData(SPI1, 0x66 );

printf("= main() : SPI Toggle NSS=PA4=->H->L \r\n" , Str_ );
   /// SPI NSS Toggle ->H->L : ///
    SPI_SSOutputCmd(SPI1, DISABLE);
     Delay_Ms(1);
    SPI_SSOutputCmd(SPI1, ENABLE);

printf("= main() : SPI send Cmd RES=0x99 \r\n" , Str_ );
    /// SPI send Cmd RES=0x99 : ///
     while( SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET )  {}
    SPI_I2S_SendData(SPI1, 0x99 );

    /// SPI NSS Set ->H = DISable : ///
printf("= main() : SPI NSS=->H \r\n\r\n" , Str_ );
    SPI_SSOutputCmd(SPI1, DISABLE);
     Delay_Ms(1);

    //

printf("= main() : SPI Toggle NSS=PA4=->H->L \r\n" , Str_ );
    /// SPI NSS Toggle ->H->L : ///
    SPI_SSOutputCmd(SPI1, DISABLE);
     Delay_Ms(1);
    SPI_SSOutputCmd(SPI1, ENABLE);

// printf("= main() : SPI send Cmd WRT=0x02 \r\n" , Str_ );
printf("= main() : SPI send Cmd WRT=0x02 :" , Str_ );
    /// SPI send Cmd WRT=0x02 : ///
     while( SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET )  {}
    SPI_I2S_SendData(SPI1, 0x02 );
// printf("= main() : SPI send Adr HIGH=00 \r\n" , Str_ );
printf(" , Adr HIGH=00 " , Str_ );
    /// SPI send Adr HIGH : ///
     while( SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET )  {}
    SPI_I2S_SendData(SPI1, 0x00 );
// printf("= main() : SPI send Adr MID=00 \r\n" , Str_ );
printf(" , Adr MID=00 " , Str_ );
    /// SPI send Adr MID : ///
     while( SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET )  {}
    SPI_I2S_SendData(SPI1, 0x00 );
// printf("= main() : SPI send Adr LOW=00 \r\n" , Str_ );
printf(" , Adr LOW=00 \r\n" , Str_ );
    /// SPI send Adr LOW : ///
     while( SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET )  {}
    SPI_I2S_SendData(SPI1, 0x00 );

printf("= main() : SPI send Str_=" );
    /// Write Str_ Chars to PSRAM : ///
    for(int i=0 ; i<Str_L ; i++ )
    {
printf("%c" , Str_[ i ] );
        /// SPI send Str_ Char : ///
         while( SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET )  {}
        SPI_I2S_SendData(SPI1, (u16)Str_[ i ] );
    }
//    printf("\r\n" );

    /// SPI NSS Set ->H = DISable : ///
printf("= main() : SPI NSS=->H \r\n\r\n" , Str_ );
    SPI_SSOutputCmd(SPI1, DISABLE);
     Delay_Ms(1);

    Delay_Ms(500);

    ;

printf("= main() : SPI Toggle PA4=->H->L \r\n" , Str_ );
    /// SPI NSS Toggle ->H->L : ///
    SPI_SSOutputCmd(SPI1, DISABLE);
     Delay_Ms(1);
    SPI_SSOutputCmd(SPI1, ENABLE);

// printf("= main() : SPI send Cmd RD=0x03 \r\n" , Str_ );
printf("= main() : SPI send Cmd RD=0x03 :" , Str_ );
    /// SPI send Cmd RD=0x03 : ///
     while( SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET )  {}
    SPI_I2S_SendData(SPI1, 0x03 );
// printf("= main() : SPI send Adr HIGH=00 \r\n" , Str_ );
printf(" , Adr HIGH=00 " , Str_ );
    /// SPI send Adr HIGH : ///
     while( SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET )  {}
    SPI_I2S_SendData(SPI1, 0x00 );
// printf("= main() : SPI send Adr MID=00 \r\n" , Str_ );
printf(" , Adr MID=00 " , Str_ );
    /// SPI send Adr MID : ///
     while( SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET )  {}
    SPI_I2S_SendData(SPI1, 0x00 );
// printf("= main() : SPI send Adr LOW=00 \r\n" , Str_ );
printf(" , Adr LOW=00 \r\n" , Str_ );
    /// SPI send Adr LOW : ///
     while( SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET )  {}
    SPI_I2S_SendData(SPI1, 0x00 );


printf("= main() : SPI read BYTE \r\n"  );
    /// TEST Read : ///
     while( SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET )  {}
    RxData_1 = SPI_I2S_ReceiveData(SPI1);
    Adr_ ++ ;
printf(" <%c=0%2X< " , RxData_1 , RxData_1 );

     while( SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET )  {}

    ;

    while( 1 )
    {

//      if(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) != RESET)
      {
          if( Adr_ % 8 == 0 )
          {
              printf("\r\n");
          }

         while( SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET )  {}
        RxData_1 = SPI_I2S_ReceiveData(SPI1);
printf(" <%c=0%2X< " , RxData_1 , RxData_1 );
         Delay_Ms(10);
         while( SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET )  {}
        Adr_ ++ ;

        if( Adr_ >= 0x40 )
        {
            Adr_ = 0 ;

printf("\r\n\r\n= main() : SPI Toggle SS=PC1,PA4=->H->L \r\n" , Str_ );
            /// SPI NSS Toggle ->H->L : ///
            SPI_SSOutputCmd(SPI1, DISABLE);
             Delay_Ms(1);
            SPI_SSOutputCmd(SPI1, ENABLE);

// printf("= main() : SPI send Cmd RD=0x03 \r\n" , Str_ );
printf("= main() : SPI send Cmd RD=0x03 : " , Str_ );
            /// SPI send Cmd RD=0x03 : ///
             while( SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET )  {}
            SPI_I2S_SendData(SPI1, 0x03 );
// printf("= main() : SPI send Adr HIGH=00 \r\n" , Str_ );
printf(" , Adr HIGH=00 " , Str_ );
                /// SPI send Adr HIGH : ///
             while( SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET )  {}
            SPI_I2S_SendData(SPI1, 0x00 );
// printf("= main() : SPI send Adr MID=00 \r\n" , Str_ );
printf(" , Adr MID=00 " , Str_ );
            /// SPI send Adr MID : ///
             while( SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET )  {}
            SPI_I2S_SendData(SPI1, 0x00 );
// printf("= main() : SPI send Adr LOW=00 \r\n" , Str_ );
printf(" , Adr LOW=00 \r\n" , Str_ );
            /// SPI send Adr LOW : ///
             while( SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET )  {}
            SPI_I2S_SendData(SPI1, 0x00 );

        }
      }

      ;
    }

    //

}

///
