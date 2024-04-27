/*
  SPI_PSRAM_BitBang_003 :

 Output :

: SPI_PSRAM_NitBang_003 :
SystemClk:48000000
ChipID:00300500
---
> Ch32_PSRAM_BitBang :
- Starting
Setup(): -> SpiRam_Reset()
Setup(): -> fill_buffer()
Setup(): -> strcpy_()
---
   SPI RAM:
- Reading
 <42=x2A< 
 Blink

Wiring :

 PSRAM Aps6404 :
--
 PA4 ->  !SS  \/  Vcc  <- 3V3
 PA6 -> MISO      DIO3   nc
 nc     DIO2      Vcc  <- PA7
 Gnd ->  Gnd      MOSI <- PA5
--

 */

/*
  Ch32v003-j4m6 :
    Pinout sop8 :
                  \/
          A6 PD6 +   + PD1 SCL
          A1 PA1 +   + PD5 A5
                     + PD4 A7
            -Gnd       PC4 A2
          A0 PA2       PC2 SCL
            +Vdd       PC1 SDA NSS RX

  Ch32v003-a4m6 :
    Pinout sop16:
      RX NSS SDA PC1  \/  PC0 NSS TX
             SCL PC2      Vdd+
                 PC3      Gnd-
              A2 PC4      PA2 A0
        SDA MOSI PC6      PA1 A1
            MISO PC7      PD7 NRST
             SCL PD1      PD6 A6
              A7 PD4      PD5 A5

  Ch32v003-f4p6 :
    Pinout :
          A7 PD4  \/   PD3 A4
          A5 PD5       PD2 A3
          A6 PD6       PD1 SCL
        NRST PD7       PC7 MISO
          A1 PA1       PC6 MOSI SDA
          A0 PA2       PC5 SCK SCL
            -Gnd       PC4 A2
     TXD SDA PD0       PC3
            +Vdd       PC2 SCL
      TX NSS PC0       PC1 SDA NSS RX

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


/* Global define */


/* Global Variable */
vu8 val;

/*
 */
void GPIO_INIT(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    /// PD4 = LED : ///
//    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(         GPIOD,      &GPIO_InitStructure);

    ;

    /// PC0/PC4 = NSS : ///
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOC, ENABLE );
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0;
//    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(         GPIOC,      &GPIO_InitStructure);

    /// PC5 = SCK : ///
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOC, ENABLE );
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(         GPIOC,      &GPIO_InitStructure);

    /// PC6 = MOSI : ///
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOC, ENABLE );
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(         GPIOC,      &GPIO_InitStructure);

    /// PC7 = MISO : ///
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOC, ENABLE );
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(         GPIOC,      &GPIO_InitStructure);

}


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
/*
int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    SystemCoreClockUpdate();
    Delay_Init();
    USART_Printf_Init(115200);
    printf("SystemClk:%d\r\n",SystemCoreClock);
    printf( "ChipID:%08x\r\n", DBGMCU_GetCHIPID() );

    USARTx_CFG();

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
}
*/

///

char  bitBangData( char _send ) ;
void  fill_buffer( int C ) ;
char  read_eeprom( long EEPROM_address ) ;
void  SpiRam_Reset() ;
void  SBB_setup() ;

//
#define  spi_transfer(data)    bitBangData(data)
//

void  strcpy_( char* T , char* F )
{
long i=0 ;

    for( i=0 ; F[ i ] != 0 ; i++ )
    {
        T[i] = F[i];
    }
    T[i] = F[i];
    T[i] = 0   ;
}


//

/*
    PC0 -> #CE=ss \/  +Vcc=3.3V -- +vdd
    PC7 <- miSO       sio3
           sio2       SCLK <- PC5
    -gnd - -Gnd       moSI <- PC6
 */

/*
//
#define  CH32_E  1
#ifndef  CH32_E  // :
 #define  DATAIN       12 //MISO
 #define  DATAOUT      11 //MOSI
 #define  SPICLOCK     13 //sck
 #define  SLAVESELECT  10 //ss
 const int  MISOPin  = 9  ;
 const int  MOSIPin  = 8  ;
 const int  SCKPin   = 10 ;
 const int  SSPin    = 11 ;
#else   // - CH32_E -
// #ifdef  CH32_E  // :
 #define  DATAIN       C7  // MISO // CH32_HW_MISO
 #define  DATAOUT      C6  // MOSI // CH32_HW_MOSI
 #define  SPICLOCK     C5  // sck  // CH32_HW_SCK
 #define  SLAVESELECT  C0  // ss   // CH32_HW_NSS
 const int  MISOPin  = C7 ;  // 9  ;
 const int  MOSIPin  = C6 ;  // 8  ;
 const int  SCKPin   = C5 ;  // 10 ;
 const int  SSPin    = C0 ;  // 11 ;
#endif  // - CH32_E .
*/

#define  MISOPort   GPIOC        // 9  ;
#define  MISOPin    GPIO_Pin_7   // 9  ;
#define  MOSIPort   GPIOC        // 8  ;
#define  MOSIPin    GPIO_Pin_6   // 8  ;
#define  SCKPort    GPIOC        // 10 ;
#define  SCKPin     GPIO_Pin_5   // 10 ;
#define  NSSPort    GPIOC        // 11 ;
#define  NSSPin     GPIO_Pin_0   // 11 ;

/// SEEP OpCodes : ///
//#define  WREN  6
//#define  WRDI  4
//#define  RDSR  5
//#define  WRSR  1
//#define  READ  3
//#define  WRITE 2

/// SPIRAM OpCodes : ///
#define  WREN  0x00  // 6
#define  WRDI   4  // 4
#define  RDSR   5  // 5
#define  WRSR   1  // 1
#define  READ  0x03  // 3
#define  WRITE 0x02  // 2
/// for Standby Mode : ///
#define  RSTE  0x66
#define  RST   0x99

// byte  eeprom_output_data;
char  eeprom_output_data;
// byte  eeprom_input_data = 0;
char  eeprom_input_data = 0;
// byte  clr;
char  clr;
long  address = 0;
/// data buffer : ///
char  buffer [256];
char  bufin[16];
char  S_[10];
//

//

/*
#define PSRAM_CMD_RES_EN     0x66
#define PSRAM_CMD_RESET      0x99
#define PSRAM_CMD_READ_ID    0x9F
#define PSRAM_KGD            0x5D

#define PSRAM_CMD_READ       0x03
#define PSRAM_CMD_READ_FAST  0x0B
#define PSRAM_CMD_WRITE      0x02

#define PSRAM_DESEL PSRAM_GPIO->BSHR = ( 1 << PSRAM_CS_PIN )
#define PSRAM_SEL   PSRAM_GPIO->BSHR = ( 1 << ( 16 + PSRAM_CS_PIN ) )
*/

//

// void  setup()
void  setup_1()
{
//  Serial.begin(    9600 );
// //  Serial.begin(  115200 );
  printf("\r\n\r\n----------\r\n> Ch32_PSRAM_BitBang : \r\n");

//  printfln("\nSPI RAM:");
  printf("- Starting\r\n\r\n");
//  delay(300);
  Delay_Ms(300);

  //

// //  pinMode(D4, OUTPUT);

  //

  //
   SBB_setup();
  //

/*
  pinMode( DATAOUT,     OUTPUT   );
  pinMode( DATAIN,      INPUT    );
  pinMode( SPICLOCK,    OUTPUT   );
*/
// //  pinMode( SLAVESELECT, OUTPUT   );
// //  digitalWrite( SLAVESELECT , HIGH ); // disable device
//   GPIO_WriteBit(GPIOC, GPIO_Pin_4, Bit_SET );
   GPIO_WriteBit( NSSPort , NSSPin , Bit_SET );
/*
*/

/*
// SPCR = 01010000
  // interrupt disabled , spi enabled , msb 1st , master , clk low when idle ,
  // sample on rising edge of clk , system clock/4 rate (fastest)
  SPCR = (1<<SPE)|(1<<MSTR);
  clr  = SPSR;
  clr  = SPDR;
  delay(10);
*/


  /// RESET SPIRAM to Initialize : - Not Needed : ///
printf("Setup(): -> SpiRam_Reset() \r\n");
  SpiRam_Reset();


  Delay_Ms(10);

  //

printf("Setup(): -> fill_buffer() \r\n");
  //fill buffer with data
//  fill_buffer();
//  fill_buffer( 0 );
  fill_buffer( ' ' );
printf("Setup(): -> strcpy_() \r\n");
//  sprintf( buffer , "*** Hello SpiRam ! ***" );
// //  strcpy( (char*)buffer , "*** Hello SpiRam ! ***" );
  strcpy_( (char*)buffer , "*** Hello SpiRam ! *** ABCdefghijklmnopqrstuvWXYZ ***" );
  //fill eeprom w/ buffer

//  ///  - Not Needed : ///
//  digitalWrite( SLAVESELECT , LOW );
//printf( "\nSetup(): SPI : write WREN = " );  printfln( WREN );
//   spi_transfer( WREN ); //write enable
//  digitalWrite( SLAVESELECT , HIGH );


  /// WRITE Buffer : ///

// //  digitalWrite( SLAVESELECT , LOW );
  GPIO_WriteBit( NSSPort , NSSPin , Bit_RESET );
#ifdef  DBG_E
printf( "\nSetup(): SPI : write Cmd WRITE = x%02X : " , WRITE );
#endif
   spi_transfer( WRITE ); //write instruction
   address = 0;
#ifdef  DBG_E
printf( " address H = x%02X " , (address>>16) & 0xFF );
#endif
   spi_transfer( (char)(address>>16) );        //send HSByte address first
#ifdef  DBG_E
printf( " M = x%02X " , ( address>>8) & 0xFF  );
#endif
   spi_transfer( (char)(address>>8) );   //send MSByte address first
#ifdef  DBG_E
printf( " L = %02X \r\n" , address & 0xFF );
#endif
   spi_transfer( (char)(address) );      //send LSByte address
   //write 128 bytes
   for( int I=0 ; I<128 ; I++ )
   {
#ifdef  DBG_E
printf( ": SPI > %s  > : " , S_ );
#endif
     spi_transfer( buffer[I] ); //write data byte
   }
// //  digitalWrite( SLAVESELECT , HIGH ); //release chip
  GPIO_WriteBit( NSSPort , NSSPin , Bit_SET );
  //wait for eeprom to finish writing
  Delay_Ms(300);

  //

  //fill buffer with data
//  fill_buffer();
  fill_buffer( -1 );
//  fill_buffer( ' ' );
//  sprintf( buffer , "*** Hello SpiRam ! ***" );
//  strcpy( buffer , "*** Hello SpiRam ! ***" );
  //fill eeprom w/ buffer

  /// WRITE Buffer : ///

//  digitalWrite( SLAVESELECT , LOW );
//printf( "\nSetup(): SPI : write WREN = " );  printfln( WREN );
//   spi_transfer( WREN ); // write enable
//  digitalWrite( SLAVESELECT , HIGH );

// //  digitalWrite( SLAVESELECT , LOW );
  GPIO_WriteBit( NSSPort , NSSPin , Bit_RESET );
#ifdef  DBG_E
printf( "\nSetup(): SPI : write cmd WRITE = %02X : " , WRITE );
#endif
   spi_transfer( WRITE ); //write instruction
   address = 128;
#ifdef  DBG_E
printf( "address : H = %x02X " , (address>>16) & 0xFF );
#endif
   spi_transfer( (char)(address>>16) );        //send HSByte address first
#ifdef  DBG_E
printf( " M = x%02X " , address>>8) & 0xFF );
#endif
   spi_transfer( (char)(address>>8) );   //send MSByte address first
#ifdef  DBG_E
printf( " L = x%02X \r\n" , address & 0xFF );
#endif
   spi_transfer( (char)(address) );      //send LSByte address

#ifdef  DBG_E
printf( "\nSetup(): SPI : Write buf[] = %s \r\n" , buffer );
#endif
   //write 128 bytes
   for( int I=0 ; I<128 ; I++ )
   {
    S_[0] = buffer[I] ;  S_[1] = 0 ;
#ifdef  DBG_E
printf( ": SPI > %s > : " , S_ );
#endif
     spi_transfer( buffer[I] ); //write data byte
   }
// //  digitalWrite( SLAVESELECT , HIGH ); //release chip
  GPIO_WriteBit( NSSPort , NSSPin , Bit_SET );
  //wait for eeprom to finish writing
  Delay_Ms(300);

  //

printf("\r\n\r\n---\nSPI RAM:\r\n");
printf("- Reading\r\n\r\n");
  Delay_Ms(300);

  address = 0;

}

//

// void  loop()
void  loop_1()
{

 while( 1 )
 {

  eeprom_output_data = read_eeprom( address );

printf( " <%c=x%02X< " , eeprom_output_data , eeprom_output_data );
// printf( " <%d=x%02X< " , eeprom_output_data , eeprom_output_data );
//  bufin[0] = eeprom_output_data;
//  bufin[1] = 0;
//  Serial.print( bufin );
//  Serial.print(" ");
// //  Serial.print("\n");

  address ++ ;
  if( address % 16 == 0 )
    printf("\r\n");

//  if( address >= 256 )
  if( address >= 64 )
  {
    address = 0 ;
    printf("\r\n");
  }

  //

//  delay(500); //pause for readability
  Delay_Ms(100); //pause for readability
//  delay(10); //pause for readability
//  delay(1000); //pause for readability

  //

printf("\r\nBlink\r\n");

// //  digitalWrite(D4, LOW);
  GPIO_WriteBit( GPIOD , GPIO_Pin_4 , Bit_RESET );
//  Delay_Ms(500);
  Delay_Ms(100);
// //  digitalWrite(D4, HIGH);
  GPIO_WriteBit( GPIOD , GPIO_Pin_4 , Bit_SET );
//   Delay_Ms(500);
   Delay_Ms(100);

//  delay(100); //pause for readability

 }

}

//

// void fill_buffer()
void  fill_buffer( int C )
{
  for (int I=0;I<128;I++)
  {
    if( C > -1 )
    {
      buffer[ I ] = C;
    }
    else
    {
      buffer[ I ] = I;
    }
  }
}


// #define  spi_transfer(data)    bitBangData(data)

/*
char  spi_transfer(volatile char data)
{
  SPDR = data;                       // Start the transmission
  while ( ! (SPSR & (1<<SPIF)) )     // Wait for the end of the transmission
  {
  }

  return( SPDR );                    // return the received byte
}
*/

char  read_eeprom( long EEPROM_address )
{
// #ifdef  DBG_E
printf( "read_eeprom(): = x%06X \r\n" , EEPROM_address );
// #endif
int data;

  /// READ EEPROM : ///

// //  digitalWrite( SLAVESELECT , LOW );
  GPIO_WriteBit( NSSPort , NSSPin , Bit_RESET );

#ifdef  DBG_E
printf( "read_eeprom(): SPI : Write READ = " );  printfln( READ );
#endif
   spi_transfer( READ ); //transmit read opcode

#ifdef  DBG_E
printf( "\nread_eeprom(): SPI : Write EEPROM_address H = " );  printfln( EEPROM_address>>16 );
#endif
   spi_transfer( (char)(EEPROM_address>>16) );  //send HSByte address first
#ifdef  DBG_E
printf( "\nread_eeprom(): SPI : Write EEPROM_address M = " );  printfln( EEPROM_address>>8 );
#endif
   spi_transfer( (char)(EEPROM_address>>8)  );   //send MSByte address first
#ifdef  DBG_E
printf( "\nread_eeprom(): SPI : Write EEPROM_address L = " );  printfln( EEPROM_address );
#endif
   spi_transfer( (char)(EEPROM_address)     );      //send LSByte address

//  data = spi_transfer( 0xFF ); //get Dummy byte

   data = spi_transfer( 0xFF ); //get data byte

// //  digitalWrite( SLAVESELECT , HIGH ); //release chip, signal end transfer
  GPIO_WriteBit( NSSPort , NSSPin , Bit_SET );

// #ifdef  DBG_E
printf( "\r\nread_eeprom(): => x%02X \r\n" , data );
// #endif
return( data );
}

void  SpiRam_Reset()
{
  /// send ENABLE RESET : ///
  spi_transfer( RSTE ); //transmit read opcode

  /// send RESET : ///
  spi_transfer( RST ); //transmit read opcode
}


///


/*
#include <stdlib.h>
*/

/*
const int  SSPin   = 11 ;
const int  SCKPin  = 10 ;
const int  MISOPin = 9  ;
const int  MOSIPin = 8  ;
*/
/*
 const int  MISOPin  = C7 ;  // 9  ;
 const int  MOSIPin  = C6 ;  // 8  ;
 const int  SCKPin   = C5 ;  // 10 ;
 const int  SSPin    = C0 ;  // 11 ;
*/

// byte  sendData  = 64;   // Value to be sent
char  sendData  = 64;   // Value to be sent
// byte  slaveData = 0;  // for storing the  value sent by the slave
char  slaveData = 0;  // for storing the  value sent by the slave


// void  setup()
void  SBB_setup()
{

    GPIO_INIT();

#ifdef  DBG_E
printfln("> SBB_setup() :");
#endif

#ifdef  DBG_E
printf( ":MISOPin = " );  printf( MISOPin );  printf( " = C7 = " );  printfln( C7 );
#endif
// //   pinMode( MISOPin , INPUT  );
#ifdef  DBG_E
printf( ":SSPin   = " );  printf( SSPin   );  printf( " = C0 = " );  printfln( C0 );
#endif
// //   pinMode( SSPin ,   OUTPUT );
#ifdef  DBG_E
printf( ":SCKPin  = " );  printf( SCKPin  );  printf( " = C5 = " );  printfln( C5 );
#endif
// //   pinMode( SCKPin ,  OUTPUT );
#ifdef  DBG_E
printf( ":MOSIPin = " );  printf( MOSIPin );  printf( " = C6 = " );  printfln( C6 );
#endif
// //   pinMode( MOSIPin , OUTPUT );
// //    digitalWrite( SSPin , HIGH );       // SS high
    GPIO_WriteBit( NSSPort , NSSPin , Bit_SET );

}


// void  loop()
void  SBB_loop()
{
// //  digitalWrite( SSPin , LOW  );         // SS low
  GPIO_WriteBit( NSSPort , NSSPin , Bit_RESET );

   slaveData = bitBangData( sendData );  // data transmission

// //  digitalWrite( SSPin , HIGH );         // SS high again
  GPIO_WriteBit( NSSPort , NSSPin , Bit_SET );
}


// byte  bitBangData( byte _send )  // This function transmit the data via bitbanging
char  bitBangData( char _send )  // This function transmit the data via bitbanging
{
char  _receive = 0 ;
#ifdef  DBG_E
printf( "\n:spi: = " );
printfln( _send );
#endif

//  for( int i=0 ; i < 8 ; i++ )  // 8 bits in a byte - Not L -> H
  for( int i=7 ; i >= 0 ; i-- )  // 8 bits in a byte - H -> L
  {

#ifdef  DBG_E
printf( " > %x%02X " , ( bitRead( _send , i ) ? HIGH : LOW ) );
#endif
// //     digitalWrite( MOSIPin      , ( bitRead( _send , i ) ? HIGH : LOW ) );  // Set MOSI
     GPIO_WriteBit( MOSIPort, MOSIPin, ( ( _send & (1 << i) ) ? SET : RESET ) );

// //    digitalWrite( SCKPin , HIGH );                         // SCK high
    GPIO_WriteBit( SCKPort , SCKPin , Bit_SET );
// //     bitWrite(     _receive , i , digitalRead( MISOPin ) ); // Capture MISO
     _receive |= ( GPIO_ReadInputDataBit( MISOPort , MISOPin ) ? 1 << i : 0 << i );
#ifdef  DBG_E
printf( " < x%02X " , digitalRead( MISOPin ) );
#endif
// //    digitalWrite( SCKPin , LOW );                          // SCK low
    GPIO_WriteBit( SCKPort , SCKPin , Bit_RESET );

  }

#ifdef  DBG_E
printf(   " => "   );
printfln( _receive );
#endif
return( _receive );        // Return the received data
}


//

int main(void)
{
// u8 i = 0;
// u8 j = 0;
// u8 value;

    SystemCoreClockUpdate();
    Delay_Init();
    USART_Printf_Init(115200);
     printf("\r\n\r\n : SPI_PSRAM_NitBang_003 : \r\n", SystemCoreClock);
     printf("SystemClk:%d\r\n", SystemCoreClock);
     printf( "ChipID:%08x\r\n", DBGMCU_GetCHIPID() );

    GPIO_INIT();

    setup_1();

    loop_1();

    while( 1 )
    {
        ;
    }

return(0);
}

///


/*

void PSRAM_SPI_WRITE( void const *buf, uint32_t size )
{
    while(size--)
        SPI_transfer_8( *(uint8_t *)(buf++) );
}

void PSRAM_SPI_READ( uint8_t *buf, uint32_t size )
{
    while(size--)
        *(buf++) = SPI_transfer_8(0);

}

uint8_t  zeros[ 3 ];

void psram_send_cmd( uint8_t cmd )
{
    PSRAM_SPI_WRITE( &cmd, 1 );
}

void psram_reset()
{
    PSRAM_SEL;
    psram_send_cmd( PSRAM_CMD_RES_EN );
    psram_send_cmd( PSRAM_CMD_RESET );
    PSRAM_DESEL;
    Delay_Ms( 10 );
}

void psram_read_id( uint8_t *dst )
{
    PSRAM_SEL;
    psram_send_cmd( PSRAM_CMD_READ_ID );
    PSRAM_SPI_WRITE( zeros, 3 );
    PSRAM_SPI_READ( dst, 6 );
    PSRAM_DESEL;
}

int psram_init()
{
    PSRAM_DESEL;
    Delay_Ms(10);

    psram_reset();

    uint8_t chipId[6];
    psram_read_id( chipId );

    if ( chipId[1] != PSRAM_KGD ) return -1;

    return 0;
}

uint8_t cmdAddr[5];

void psram_write( uint32_t addr, void const *ptr, uint32_t size )
{
    cmdAddr[0] = PSRAM_CMD_WRITE;

    cmdAddr[1] = ( addr >> 16 ) & 0xff;
    cmdAddr[2] = ( addr >> 8 ) & 0xff;
    cmdAddr[3] = addr & 0xff;

    PSRAM_SEL;
    PSRAM_SPI_WRITE( cmdAddr, 4 );
    PSRAM_SPI_WRITE( ptr, size );
    PSRAM_DESEL;
}

void psram_read( uint32_t addr, void *ptr, uint32_t size )
{
    cmdAddr[0] = PSRAM_CMD_READ;

    cmdAddr[1] = ( addr >> 16 ) & 0xff;
    cmdAddr[2] = ( addr >> 8 ) & 0xff;
    cmdAddr[3] = addr & 0xff;

    PSRAM_SEL;
    PSRAM_SPI_WRITE( cmdAddr, 4 );
    PSRAM_SPI_READ( ptr, size );
    PSRAM_DESEL;
}

void psram_load_data( void const *buf, uint32_t addr, uint32_t size )
{
    while ( size >= 64 )
    {
        psram_write( addr, buf, 64 );
        addr += 64;
        buf  += 64;
        size -= 64;
    }

    if ( size ) psram_write( addr, buf, size );
}

void psram_read_data( void *buf, uint32_t addr, uint32_t size )
{
    while ( size >= 64 )
    {
        psram_read( addr, buf, 64 );
        addr += 64;
        buf += 64;
        size -= 64;
    }

    if ( size ) psram_read( addr, buf, size );
}

*/

///
