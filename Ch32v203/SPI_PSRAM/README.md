This seems close to working.

I am getting cms / data back and forth , but only Reading 0 zeroes from hip.

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
... 
etc