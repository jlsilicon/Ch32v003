#ifndef HW_SPI_H
#define HW_SPI_H

// SD_CS_PIN   : PC0/PC3 : SD Card CS Push-Pull
// SD_SCK_PIN  : SCK on PC5, 10MHz Output, alt func, push-pull
// SD_MOSI_PIN : MOSI on PC6, 10MHz Output, alt func, push-pull
// SD_MISO_PIN : MISO on PC7, 10MHz input, floating

void SPI_init();
void SPI_begin_8();
void SPI_end();
uint8_t SPI_transfer_8(uint8_t data);
void SPI_set_prescaler(uint8_t presc);

#endif
