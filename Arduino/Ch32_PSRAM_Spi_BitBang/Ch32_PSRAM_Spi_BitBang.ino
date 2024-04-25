
/// : CH32 : ch32_PSRAM_Spi_BitBang.ino : ///

/*
  - uses Aps6404 / Ips6404 
  - compatible with SEEP chips 

  Aps6404 / Ips6404 Pinout -> Arduino Pins : 
    PC0 -> #CE=ss \/  +Vcc=3.3V -- +vdd 
    PC7 <- miSO       sio3 
           sio2       SCLK <- PC5 
    -gnd - -Gnd       moSI <- PC6 

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

///

// #define  DBG_E  1  


#include <stdlib.h>

//
 #define  spi_transfer(data)    bitBangData(data)  
//

//

/*
    PC0 -> #CE=ss \/  +Vcc=3.3V -- +vdd 
    PC7 <- miSO       sio3 
           sio2       SCLK <- PC5 
    -gnd - -Gnd       moSI <- PC6 
 */

//
#define  CH32_E  1  
#ifndef  CH32_E  // : 
 #define  SLAVESELECT  11 //ss
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

byte  eeprom_output_data;
byte  eeprom_input_data = 0;
byte  clr;
long  address = 0;
/// data buffer : ///
char  buffer [128];
char  bufin[16];
char  S_[10];

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

void  setup()
{
//  Serial.begin(    9600 );
  Serial.begin(  115200 );
  Serial.println("\n\n----------\n\n> Ch32_PSRAM_BitBang :");

//  Serial.println("\nSPI RAM:");
  Serial.println("- Starting");
  Serial.println("");
  delay(300);

  //

  pinMode(D4, OUTPUT);

  //

  //
   SBB_setup();
  //

/*
  pinMode( DATAOUT,     OUTPUT   );
  pinMode( DATAIN,      INPUT    );
  pinMode( SPICLOCK,    OUTPUT   );
*/
  pinMode( SLAVESELECT, OUTPUT   );
  digitalWrite( SLAVESELECT , HIGH ); // disable device
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
Serial.println("Setup(): -> SpiRam_Reset()");
  SpiRam_Reset();


  delay(10);

  //

  //fill buffer with data
//  fill_buffer( 0 );
  fill_buffer( ' ' );
//  sprintf( buffer , "*** Hello SpiRam ! ***" );
  strcpy( buffer , "*** Hello SpiRam ! ***" );
  //fill eeprom w/ buffer

//  ///  - Not Needed : ///
//  digitalWrite( SLAVESELECT , LOW );
//Serial.print( "\nSetup(): SPI : write WREN = " );  Serial.println( WREN );
//   spi_transfer( WREN ); //write enable
//  digitalWrite( SLAVESELECT , HIGH );


  /// WRITE Buffer : ///

  digitalWrite( SLAVESELECT , LOW );
#ifdef  DBG_E 
Serial.print( "\nSetup(): SPI : write WRITE = " );  Serial.println( WRITE );
#endif
   spi_transfer( WRITE ); //write instruction
   address = 0;
#ifdef  DBG_E 
Serial.print( "\nSetup(): SPI : Write address H = " );  Serial.println( address>>16 );
#endif
   spi_transfer( (char)(address>>16) );        //send HSByte address first
#ifdef  DBG_E 
Serial.print( "\nSetup(): SPI : Write address H = " );  Serial.println( address>>8 );
#endif
   spi_transfer( (char)(address>>8) );   //send MSByte address first
#ifdef  DBG_E 
Serial.print( "\nSetup(): SPI : Write address H = " );  Serial.println( address );
#endif
   spi_transfer( (char)(address) );      //send LSByte address
   //write 128 bytes
   for( int I=0 ; I<128 ; I++ )
   {
#ifdef  DBG_E 
Serial.print( ": SPI > " );  Serial.print( S_ );  Serial.print( " > : " );
#endif
     spi_transfer( buffer[I] ); //write data byte
   }
  digitalWrite( SLAVESELECT , HIGH ); //release chip
  //wait for eeprom to finish writing
  delay(300);

  //

  //fill buffer with data
  fill_buffer( -1 );
//  fill_buffer( ' ' );
//  sprintf( buffer , "*** Hello SpiRam ! ***" );
//  strcpy( buffer , "*** Hello SpiRam ! ***" );
  //fill eeprom w/ buffer

  /// WRITE Buffer : ///

//  digitalWrite( SLAVESELECT , LOW );
//Serial.print( "\nSetup(): SPI : write WREN = " );  Serial.println( WREN );
//   spi_transfer( WREN ); // write enable
//  digitalWrite( SLAVESELECT , HIGH );
  
  digitalWrite( SLAVESELECT , LOW );
#ifdef  DBG_E 
Serial.print( "\nSetup(): SPI : write WRITE = " );  Serial.println( WRITE );
#endif
   spi_transfer( WRITE ); //write instruction
   address = 128;
#ifdef  DBG_E 
Serial.print( "\nSetup(): SPI : Write address H = " );  Serial.println( address>>16 );
#endif
   spi_transfer( (char)(address>>16) );        //send HSByte address first
#ifdef  DBG_E 
Serial.print( "\nSetup(): SPI : Write address M = " );  Serial.println( address>>8 );
#endif
   spi_transfer( (char)(address>>8) );   //send MSByte address first
#ifdef  DBG_E 
Serial.print( "\nSetup(): SPI : Write address L = " );  Serial.println( address );
#endif
   spi_transfer( (char)(address) );      //send LSByte address

#ifdef  DBG_E 
Serial.print( "\nSetup(): SPI : Write buf[] = " );  Serial.println( buffer );
#endif
   //write 128 bytes
   for( int I=0 ; I<128 ; I++ )
   {
    S_[0] = buffer[I] ;  S_[1] = 0 ;  
#ifdef  DBG_E 
Serial.print( ": SPI > " );  Serial.print( S_ );  Serial.print( " > : " );
#endif
     spi_transfer( buffer[I] ); //write data byte
   }
  digitalWrite( SLAVESELECT , HIGH ); //release chip
  //wait for eeprom to finish writing
  delay(300);

  //

  Serial.println("\n\n---\nSPI RAM:");
  Serial.println("- Reading");
  Serial.println("");
  delay(300);

  address = 0;

}

//

void  loop()
{

  eeprom_output_data = read_eeprom( address );

  Serial.print( eeprom_output_data );
  Serial.print(" ");
  bufin[0] = eeprom_output_data;
  bufin[1] = 0;
  Serial.print( bufin );
  Serial.print(" ");
//  Serial.print("\n");

  address ++ ;
  if( address % 16 == 0 )
    Serial.print("\n");

  if( address >= 256 )
  {
    address = 0 ;
    Serial.print("\n");
  }

  //
  
  delay(100); //pause for readability

  //

  Serial.println("Blink");

  digitalWrite(D4, LOW);
   delay(500);
  digitalWrite(D4, HIGH);
   delay(500);

}

//

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

byte  read_eeprom( long EEPROM_address )
{
#ifdef  DBG_E 
Serial.print( "read_eeprom(): = " );  Serial.println( EEPROM_address );
#endif
int data;
  
  /// READ EEPROM : ///

  digitalWrite( SLAVESELECT , LOW );

#ifdef  DBG_E 
Serial.print( "read_eeprom(): SPI : Write READ = " );  Serial.println( READ );
#endif
   spi_transfer( READ ); //transmit read opcode

#ifdef  DBG_E 
Serial.print( "\nread_eeprom(): SPI : Write EEPROM_address H = " );  Serial.println( EEPROM_address>>16 );
#endif
   spi_transfer( (char)(EEPROM_address>>16) );  //send HSByte address first
#ifdef  DBG_E 
Serial.print( "\nread_eeprom(): SPI : Write EEPROM_address M = " );  Serial.println( EEPROM_address>>8 );
#endif
   spi_transfer( (char)(EEPROM_address>>8)  );   //send MSByte address first
#ifdef  DBG_E 
Serial.print( "\nread_eeprom(): SPI : Write EEPROM_address L = " );  Serial.println( EEPROM_address );
#endif
   spi_transfer( (char)(EEPROM_address)     );      //send LSByte address

//  data = spi_transfer( 0xFF ); //get Dummy byte

   data = spi_transfer( 0xFF ); //get data byte

  digitalWrite( SLAVESELECT , HIGH ); //release chip, signal end transfer

#ifdef  DBG_E 
Serial.print( "\nread_eeprom(): => " );  Serial.println( data );
#endif
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

byte  sendData  = 64;   // Value to be sent
byte  slaveData = 0;  // for storing the  value sent by the slave


void  SBB_setup()
{
#ifdef  DBG_E 
Serial.println("> SBB_setup() :");
#endif

#ifdef  DBG_E 
Serial.print( ":MISOPin = " );  Serial.print( MISOPin );  Serial.print( " = C7 = " );  Serial.println( C7 );  
#endif
   pinMode( MISOPin , INPUT  );
#ifdef  DBG_E 
Serial.print( ":SSPin   = " );  Serial.print( SSPin   );  Serial.print( " = C0 = " );  Serial.println( C0 );  
#endif
   pinMode( SSPin ,   OUTPUT );
#ifdef  DBG_E 
Serial.print( ":SCKPin  = " );  Serial.print( SCKPin  );  Serial.print( " = C5 = " );  Serial.println( C5 );  
#endif
   pinMode( SCKPin ,  OUTPUT );
#ifdef  DBG_E 
Serial.print( ":MOSIPin = " );  Serial.print( MOSIPin );  Serial.print( " = C6 = " );  Serial.println( C6 );  
#endif
   pinMode( MOSIPin , OUTPUT );
    digitalWrite( SSPin , HIGH );       // SS high 

}


// Test :
void  SBB_loop()
{
  digitalWrite( SSPin , LOW  );         // SS low
   slaveData = bitBangData( sendData );  // data transmission
  digitalWrite( SSPin , HIGH );         // SS high again 
}


byte  bitBangData( byte _send )  // This function transmit the data via bitbanging
{
byte  _receive = 0 ;
#ifdef  DBG_E 
Serial.print( "\n:spi: = " );
Serial.println( _send );
#endif

  for( int i=7 ; i >= 0 ; i-- )  // 8 bits in a byte - H -> L 
  {

#ifdef  DBG_E 
Serial.print( " >" );  Serial.print( ( bitRead( _send , i ) ? HIGH : LOW ) );
#endif
     digitalWrite( MOSIPin      , ( bitRead( _send , i ) ? HIGH : LOW ) );  // Set MOSI

    digitalWrite( SCKPin , HIGH );                         // SCK high
     bitWrite(     _receive , i , digitalRead( MISOPin ) ); // Capture MISO
#ifdef  DBG_E 
Serial.print( " <" );  Serial.print( digitalRead( MISOPin ) );
#endif
    digitalWrite( SCKPin , LOW );                          // SCK low

  } 

#ifdef  DBG_E 
Serial.print(   " => "   );
Serial.println( _receive );
#endif
return( _receive );        // Return the received data
}

///
