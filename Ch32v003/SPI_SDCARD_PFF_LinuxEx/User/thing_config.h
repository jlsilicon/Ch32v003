#ifndef _THING_CONFIG_H
#define _THING_CONFIG_H

 #include "debug.h"

// // #include "ch32v003fun.h"

// Image filename
 // // #define IMAGE_FILENAME "IMAGE"
 #define IMAGE_FILENAME "testfil.txt"

// // // PSRAM CS
// // #define PSRAM_GPIO GPIOD
// // #define PSRAM_CS_PIN 3

// SD_CS_PIN : SD CS : PC0 / PC3 :
#define SD_CS_GPIO GPIOC
// #define SD_CS_PIN 0
 #define SD_CS_PIN 3

// RAM size in megabytes
#define EMULATOR_RAM_MB 8

// Keyboard queue length
#define KEY_QUEUE_LEN 16


 int  main_2() ;
 // int  ram_write( char* addr, char* buff, int* br ) ;
 // int  ram_write( uint32_t addr, uint32_t buff, int* br ) ;
 int  ram_write( uint32_t addr, char* buff, int* br ) ;



#endif
