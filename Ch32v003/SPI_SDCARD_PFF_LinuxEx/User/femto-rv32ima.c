

// PC0/PC1 : SD Card CS Push-Pull
// SCK on PC5, 10MHz Output, alt func, push-pull
// MOSI on PC6, 10MHz Output, alt func, push-pull
// MISO on PC7, 10MHz input, floating

// #define IMAGE_FILENAME "testfil.txt"

//

 #include "debug.h"

// // #include "ch32v003fun.h"

#include <stdio.h>

// // #include "emulator.h"
#include "hw_spi.h"
// // #include "pff/pff.h"
 #include "pff.h"
// .. #include "psram.h"
#include "thing_config.h"


 #define  RAM_BUFF_SIZE  1000
 char  RamBuf[ RAM_BUFF_SIZE ];
 /// copy to addr from buff and return br = size : ///
 // int  ram_write( char* addr, char* buff, int* br )
 int  ram_write( uint32_t addr, char* buff, int* br )
 {
 int  i;

 //    (* br) = strncpy( RamBuf + addr , buff ) ;
     for( i=0 ; i < RAM_BUFF_SIZE && buff[ i ] != 0 ; i++ )
     {
         RamBuf[ (long)addr + i ] = buff[ i ] ;
     }
     RamBuf[ (long)addr + i ] = 0 ;
     i++ ;

 return( i ) ;
 }


void load_sd_file( uint32_t addr, const char filename[] );

int main_2()
{
	SystemInit();
	
	// Enable GPIOs
	RCC->APB2PCENR |= RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOC;

// //	// PSRAM CS Push-Pull
// //	PSRAM_GPIO->CFGLR &= ~( 0xf << ( 4 * PSRAM_CS_PIN ) );
// //	PSRAM_GPIO->CFGLR |= ( GPIO_Speed_50MHz | GPIO_CNF_OUT_PP ) << ( 4 * PSRAM_CS_PIN );
// //	PSRAM_GPIO->BSHR = ( 1 << PSRAM_CS_PIN );

	// PC0/PC1 : SD Card CS Push-Pull
	SD_CS_GPIO->CFGLR &= ~( 0xf << ( 4 * SD_CS_PIN ) );
// //    SD_CS_GPIO->CFGLR |= ( GPIO_Speed_50MHz | GPIO_CNF_OUT_PP ) << ( 4 * SD_CS_PIN );
     SD_CS_GPIO->CFGLR |= ( GPIO_Speed_50MHz | GPIO_Mode_Out_PP ) << ( 4 * SD_CS_PIN );
	SD_CS_GPIO->BSHR = ( 1 << SD_CS_PIN );

	// Enable UART receive and receive interrupt
	USART1->CTLR1 |= USART_CTLR1_RE | USART_CTLR1_RXNEIE;
	NVIC_EnableIRQ(USART1_IRQn);

	// Enable SPI
	SPI_init();
	SPI_begin_8();

	Delay_Ms( 2000 );

// //	printf("\033[2J\033[1;1H"); // clear terminal
// //	printf("\n\rfemto-rv32ima, compiled %s\n\r", __TIMESTAMP__);

// //	if ( psram_init() )
// //	{
// //		printf( "PSRAM init failed!\n\r" );
// //		while ( 1 )
// //			;
// //	}
// //	else printf( "%d MB PSRAM OK!\n\r", EMULATOR_RAM_MB);

	load_sd_file( 0, IMAGE_FILENAME );

// //	puts( "\nStarting RISC-V VM\n\n\r" );
// //	int c = riscv_emu();
// //	while ( c == EMU_REBOOT ) c = riscv_emu();

	for( ;; )
		;

return(0);
}

// UART queue
volatile char key_queue[KEY_QUEUE_LEN + 1]; 
volatile uint8_t keys_num;

// UART receive interrupt handler
void USART1_IRQHandler( void ) __attribute__((interrupt));
void USART1_IRQHandler( void )
{
	if(keys_num <= KEY_QUEUE_LEN)
		key_queue[keys_num++] =  USART1->DATAR;
}

// SD card file loading functions
void die( FRESULT rc )
{
	printf( "Failed with rc=%u.\n", rc );
	for ( ;; )
		;
}

void load_sd_file( uint32_t addr, const char filename[] )
{
	FATFS fatfs; /* File system object */
	UINT br;
	BYTE buff[64];

printf( "Mounting volume.\n\r" );
	FRESULT rc = pf_mount( &fatfs );
	if ( rc ) die( rc );

printf( "Opening file \"%s\"\n\r", filename );
	rc = pf_open( filename );
	if ( rc ) die( rc );

printf( "Loading image into RAM : \n\r" );

	uint32_t total_bytes = 0;
	uint8_t cnt = 0;
	const char spinner[] = "/-\\|";

	for ( ;; )
	{
		rc = pf_read( buff, sizeof( buff ), &br ); /* Read a chunk of file */
		if ( rc || ! br ) break; /* Error or end of file */

// //        psram_write( addr, buff, br );
        ram_write( addr, buff, & br );
printf( "\r\n> SD Read : %s \r\n" , buff );

        total_bytes += br;
		addr += br;

		if(total_bytes % (16*1024) == 0)
		{
			cnt ++ ;
printf("%d kb so far...  ", total_bytes/1024);
			putchar(spinner[cnt%4]);
			putchar('\r');
		}

	}
	if ( rc )
	    die( rc );

printf("\n\rLoaded %d kilobytes.\n\r", total_bytes/1024);
}

///
