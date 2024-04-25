/*

  - need to plugin USB to Computer
  - look for Drive in FileManager

- COMPILES , Downloads , WORKS
*/

#include <SPI_FLASH.h>
#include <SW_UDISK.h>
#include "debug.h"
#include "usb_lib.h"
#include "Internal_Flash.h"

/* @Note
 * UDisk Example:
 * This program provides examples of UDisk.Supports external SPI Flash and internal
 * Flash, selected by STORAGE_MEDIUM at SW_UDISK.h.
 *  */

/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
int main(void)
{   
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	SystemCoreClockUpdate();
	Delay_Init(); 
    USART_Printf_Init(115200);
	printf("SystemClk:%d\r\n",SystemCoreClock);
	printf( "ChipID:%08x\r\n", DBGMCU_GetCHIPID() );
#if (STORAGE_MEDIUM == MEDIUM_SPI_FLASH)
    printf("USBD UDisk Demo\nStorage Medium: SPI FLASH \r\n");
	/* SPI flash init */
    FLASH_Port_Init( );
    /* FLASH ID check */
    FLASH_IC_Check( );
#elif (STORAGE_MEDIUM == MEDIUM_INTERAL_FLASH)
    printf("USBD UDisk Demo\r\nStorage Medium: INTERNAL FLASH \r\n");
    Flash_Sector_Count = IFLASH_UDISK_SIZE  / DEF_UDISK_SECTOR_SIZE;
    Flash_Sector_Size = DEF_UDISK_SECTOR_SIZE;
#endif

    /* Enable Udisk */
    Udisk_Capability = Flash_Sector_Count;
    Udisk_Status |= DEF_UDISK_EN_FLAG;

    /* USBD init */
    Set_USBConfig();
    USB_Init();
    USB_Interrupts_Config();

    while(1)
    {
        ;
    }
}







