This is a working version of accessing PSRAM via SPI using Bitbangig.

Wiring :

     PSRAM Aps6404 :
    --
     PA4 ->  !SS  \/  Vcc  <- 3V3
     PA6 -> MISO      DIO3   nc
     nc     DIO2      Vcc  <- PA7
     Gnd ->  Gnd      MOSI <- PA5
    --

