/*

  Create project as : ch32v003 - f4p6  (- Not 103/203)
  - copy main.c from /EVT/EXAM/.../I2C_mpu6050/User/

 FAQs :
 - Failed Download : popped off SWDIO etc pin , menu:Configure Wrong Chip
 - Typo used id 0x58 , should be 0x68
 - Wrong I2C pins for 003 : not PA1=SDA PA2=SCL , should be PC1=SDA PC2=SCL
 - Compass Writing to Wrong Address - just Adr 0x00
     and setMode : MechaQMC5883_WriteReg(): ID[x0D] : [x09] = x1D

The VCC and GND pins of the CH32V103 development board are connected to the VCC and GND pins of the MPU6050 module.
 PC1 = SDA : The PC1 pin of the CH32V003 development board is connected to the SDA pin of the MPU6050 module
 PC2 = SCL : The PC2 pin of the CH32V003 development board is connected to the SCL pin of the MPU6050 module

//The VCC and GND pins of the CH32V103 development board are connected to the VCC and GND pins of the MPU6050 module.
// PA1 = SDA : The PA1 pin of the CH32V103 development board is connected to the SDA pin of the MPU6050 module
// PA2 = SCL : The PA2 pin of the CH32V103 development board is connected to the SCL pin of the MPU6050 module

Mpu6050 : Id Address = 0x68 wrt , 0x69 rd , 0x68<<1

 - Compiles Downloads , Returns same data : Temp=175, AX=-11823 , AY=-11823 , AZ=-11823 , GX=-11823 , AGY=-11823 , GZ=-11823

 - TX = PA9
 - RX = PA10

- WORKS -
*/
/*

Output :


: MPU6050 003 :
- SystemClk:48000000
> IIC_Init() : bound=100000 , address=68 :
< IIC_Init() .
> MPU6050_Write() :
= MPU6050_Write() -> MPU_Set_Gyro_Fsr()
= MPU6050_Write() -> MPU_Set_Accel_Fsr()
= MPU6050_Write() -> MPU_Set_Rate()
= MPU6050_Write() ...
< MPU6050_Write() .
> mpu6050 Test ...

> SNOOP : Scan Devices :

* dvc69: r00 = FF  ,  dvc69: r75 = 68
* dvc6B: r00 = FF  ,  dvc6B: r75 = 68
 * dvc6D: r00 = FF  D: r75 = 68
* dvc6F: r00 = FF  ,  dvc6F: r75 = 68
* dvc71: r00 = FF  ,  dvc71: r75 = 68

* dvc97: r00 = FF  ,  dvc97: r75 = 68
* dvc99: r00 = FF  ,  dvc99: r75 = 68
 * dvc9B: r00 = FF  ,  dv68
* dvc9D: r00 = FF  ,  dvc9D: r75 = 68
* dvc9F: r00 = FF  ,  dvc9F: r75 = 68

* dvcC9: r00 = FF  ,  dvcC9: r75 = 68
* dvcCB: r00 = FF  ,  dvcCB: r75 = 68
 * dvcCD:  r00 = FF  ,  dvcCD: r75 = 68
* dvcCF: r00 = FF  ,  dvcCF: r75 = 68
* dvcD1: r00 = FF  ,  dvcD1: r75 = 68

<


ACC  : X=17228 , Y=-490 , Z=1082
GYRO : X=-363 , Y=-81 , Z=-141

 Temperature:2745
ACC  : X=17336 , Y=-450 , Z=1076
GYRO : X=-385 , Y=-89 , Z=-232

 Temperature:2752
ACC  : X=17652 , Y=-666 , Z=1096
GYRO : X=-953 , Y=-364 , Z=-789

 Temperature:2760
ACC  : X=17196 , Y=-442 , Z=1074
GYRO : X=-334 , Y=-30 , Z=-71

 Temperature:2764
ACC  : X=17170 , Y=2434 , Z=1196
GYRO : X=-788 , Y=-640 , Z=-5729

 Temperature:2766
ACC  : X=15506 , Y=6944 , Z=1336
GYRO : X=-1266 , Y=-315 , Z=-4543

 Temperature:2770
ACC  : X=14536 , Y=8802 , Z=1432
GYRO : X=-326 , Y=53 , Z=-2003


 Temperature:2755
ACC  : X=17648 , Y=-1382 , Z=-2266
GYRO : X=98 , Y=-879 , Z=-431

 Temperature:2765
ACC  : X=17158 , Y=-480 , Z=-4640
GYRO : X=-238 , Y=-2639 , Z=-459

 Temperature:2771
ACC  : X=16138 , Y=-938 , Z=-7282
GYRO : X=537 , Y=-3935 , Z=-489

 Temperature:2773
ACC  : X=13872 , Y=-1016 , Z=-10700
GYRO : X=-203 , Y=-2806 , Z=-1125

 Temperature:2776
ACC  : X=12804 , Y=-118 , Z=-12634
GYRO : X=-376 , Y=-1719 , Z=-1261

 Temperature:2779
ACC  : X=15312 , Y=-880 , Z=-8602
GYRO : X=805 , Y=6467 , Z=1711

 Temperature:2783
ACC  : X=15758 , Y=-3340 , Z=-5560
GYRO : X=-198 , Y=-438 , Z=1705

 Temperature:2785
ACC  : X=16772 , Y=-2438 , Z=-5098
GYRO : X=339 , Y=2083 , Z=1109

 Temperature:2786
ACC  : X=16914 , Y=-3206 , Z=-2504
GYRO : X=-550 , Y=764 , Z=-183

 Temperature:2789
ACC  : X=17258 , Y=-3142 , Z=-3218
GYRO : X=-353 , Y=-206 , Z=-299

 Temperature:2794
ACC  : X=17126 , Y=-2578 , Z=-2918
GYRO : X=-1507 , Y=-640 , Z=-1084

 Temperature:2799
ACC  : X=17176 , Y=736 , Z=-2528
GYRO : X=-971 , Y=-499 , Z=-2939

 Temperature:2802
ACC  : X=17278 , Y=2776 , Z=-2682
GYRO : X=-605 , Y=28 , Z=-1923

 Temperature:2805
ACC  : X=16264 , Y=4588 , Z=-2324
GYRO : X=-404 , Y=-362 , Z=568


 Temperature:2788
ACC  : X=17156 , Y=-2930 , Z=-2542
GYRO : X=-1090 , Y=113 , Z=196


ACC  : X=17554 , Y=-1932 , Z=-2094
GYRO : X=586 , Y=-1509 , Z=2648

 Temperature:2723
ACC  : X=17380 , Y=-1980 , Z=-2290
GYRO : X=-250 , Y=-57 , Z=-1

 Temperature:2733
ACC  : X=17630 , Y=582 , Z=-1344
GYRO : X=919 , Y=555 , Z=-2866

 Temperature:2739
ACC  : X=16100 , Y=4330 , Z=-2050
GYRO : X=-917 , Y=-948 , Z=-4507

 Temperature:2742
ACC  : X=15060 , Y=7624 , Z=-2274
GYRO : X=-931 , Y=-88 , Z=-2531

 Temperature:2745
ACC  : X=14334 , Y=8134 , Z=-2348
GYRO : X=290 , Y=0 , Z=4672


ACC  : X=17540 , Y=1062 , Z=-660
GYRO : X=-725 , Y=169 , Z=-258

 Temperature:2738
ACC  : X=17246 , Y=2284 , Z=-128
GYRO : X=-1049 , Y=-206 , Z=-250

 Temperature:2745
ACC  : X=17560 , Y=1094 , Z=-802
GYRO : X=-190 , Y=-333 , Z=-2371

 Temperature:2752
ACC  : X=15522 , Y=6718 , Z=-1532
GYRO : X=-1233 , Y=-881 , Z=-5025

 Temperature:2758
ACC  : X=12136 , Y=11176 , Z=-1462
GYRO : X=-1309 , Y=-569 , Z=-5168

 Temperature:2762
ACC  : X=9374 , Y=13846 , Z=-1524
GYRO : X=-1167 , Y=-468 , Z=-2788

 Temperature:2764
ACC  : X=9766 , Y=12714 , Z=-2386
GYRO : X=757 , Y=-981 , Z=3324

 Temperature:2769
ACC  : X=14338 , Y=9206 , Z=-3278
GYRO : X=1002 , Y=-456 , Z=2208

 Temperature:2773
ACC  : X=16182 , Y=5124 , Z=-3054
GYRO : X=347 , Y=890 , Z=3239

 Temperature:2774
ACC  : X=17586 , Y=-76 , Z=-1722
GYRO : X=1395 , Y=713 , Z=5533


 Temperature:2792
ACC  : X=16948 , Y=-4276 , Z=-1744
GYRO : X=-36 , Y=-1080 , Z=2626

 Temperature:2796
ACC  : X=14582 , Y=-9724 , Z=-1482
GYRO : X=2475 , Y=-537 , Z=4286

 Temperature:2801
ACC  : X=10942 , Y=-13008 , Z=-558
GYRO : X=1316 , Y=-601 , Z=5066

 Temperature:2805
ACC  : X=8694 , Y=-14914 , Z=-278
GYRO : X=-117 , Y=-88 , Z=293

 Temperature:2809
ACC  : X=6540 , Y=-15778 , Z=314
GYRO : X=589 , Y=-556 , Z=2217

 Temperature:2813
ACC  : X=5366 , Y=-16216 , Z=654
GYRO : X=102 , Y=-407 , Z=720

 Temperature:2817
ACC  : X=7750 , Y=-14754 , Z=-2234
GYRO : X=-6157 , Y=-2149 , Z=-2878

 Temperature:2826
ACC  : X=11744 , Y=-7016 , Z=-11324
GYRO : X=-5323 , Y=-4320 , Z=-3759

 Temperature:2832
ACC  : X=9602 , Y=-2234 , Z=-14698
GYRO : X=-2110 , Y=-5370 , Z=-2714

 Temperature:2835
ACC  : X=3388 , Y=522 , Z=-17144
GYRO : X=-982 , Y=-4714 , Z=-2141

 Temperature:2839
ACC  : X=-594 , Y=1308 , Z=-17222
GYRO : X=-129 , Y=-1672 , Z=-933

 Temperature:2842
ACC  : X=-2078 , Y=1736 , Z=-16932
GYRO : X=-544 , Y=-2213 , Z=-938

 Temperature:2842
ACC  : X=-2782 , Y=1818 , Z=-16780
GYRO : X=-1355 , Y=109 , Z=-115

 Temperature:2846
ACC  : X=-1748 , Y=2996 , Z=-16712
GYRO : X=-679 , Y=1110 , Z=15

 Temperature:2848
ACC  : X=8994 , Y=676 , Z=-14522
GYRO : X=345 , Y=11567 , Z=62

 Temperature:2849
ACC  : X=14846 , Y=292 , Z=-10048
GYRO : X=804 , Y=3665 , Z=-53

 Temperature:2850
ACC  : X=16938 , Y=-108 , Z=-4118
GYRO : X=-2373 , Y=7016 , Z=1027


Compass Output : - Not changing => QMC5883 ?? :

 Temperature:2755
ACC  : X=17294 , Y=892 , Z=-2852
GYRO : X=-348 , Y=-137 , Z=-203
 Id=68   cX=16717  cY=-773  cZ=-19205

 Temperature:2754
ACC  : X=17152 , Y=920 , Z=-2814
GYRO : X=-364 , Y=-72 , Z=-291
 Id=68   cX=16717  cY=-773  cZ=-19205


CompassOutput - FIXED :

 Temperature:2790
 Id=FF > MechaQMC5883_WriteReg(): ID[x0D] : [x0B] = x01
> MechaQMC5883_WriteReg(): ID[x0D] : [x09] = x1D
- cx=1950 , cy=-2033 , cz=343
ACC  : X=17288 , Y=-582 , Z=-1830
GYRO : X=-329 , Y=180 , Z=-167

 Temperature:2790
 Id=FF > MechaQMC5883_WriteReg(): ID[x0D] : [x0B] = x01
> MechaQMC5883_WriteReg(): ID[x0D] : [x09] = x1D
- cx=1971 , cy=-2023 , cz=370
ACC  : X=17404 , Y=-380 , Z=-1638
GYRO : X=-394 , Y=-303 , Z=-198

 Temperature:2791
 Id=FF > MechaQMC5883_WriteReg(): ID[x0D] : [x0B] = x01
> MechaQMC5883_WriteReg(): ID[x0D] : [x09] = x1D
- cx=1958 , cy=-2036 , cz=353
ACC  : X=17444 , Y=-234 , Z=-3238
GYRO : X=458 , Y=-2224 , Z=-1164

 Temperature:2791
 Id=FF > MechaQMC5883_WriteReg(): ID[x0D] : [x0B] = x01
> MechaQMC5883_WriteReg(): ID[x0D] : [x09] = x1D
- cx=1950 , cy=-2115 , cz=233
ACC  : X=16756 , Y=420 , Z=-5896
GYRO : X=-872 , Y=-2016 , Z=-1908

 Temperature:2791
 Id=FF > MechaQMC5883_WriteReg(): ID[x0D] : [x0B] = x01
> MechaQMC5883_WriteReg(): ID[x0D] : [x09] = x1D
- cx=1840 , cy=-2212 , cz=43
ACC  : X=15070 , Y=-492 , Z=-10420
GYRO : X=-218 , Y=-3083 , Z=-751

 Temperature:2793
 Id=FF > MechaQMC5883_WriteReg(): ID[x0D] : [x0B] = x01
> MechaQMC5883_WriteReg(): ID[x0D] : [x09] = x1D
- cx=1781 , cy=-2361 , cz=-336
ACC  : X=12162 , Y=912 , Z=-13250
GYRO : X=-429 , Y=-2588 , Z=-551

 Temperature:2793
 Id=FF > MechaQMC5883_WriteReg(): ID[x0D] : [x0B] = x01
> MechaQMC5883_WriteReg(): ID[x0D] : [x09] = x1D
- cx=1666 , cy=-2368 , cz=-658
ACC  : X=9758 , Y=1872 , Z=-14832
GYRO : X=113 , Y=-2959 , Z=-658

 Temperature:2791
 Id=FF > MechaQMC5883_WriteReg(): ID[x0D] : [x0B] = x01
> MechaQMC5883_WriteReg(): ID[x0D] : [x09] = x1D
- cx=1562 , cy=-2332 , cz=-921
ACC  : X=8356 , Y=1820 , Z=-15476
GYRO : X=-24 , Y=-1245 , Z=-610

 Temperature:2792
 Id=FF > MechaQMC5883_WriteReg(): ID[x0D] : [x0B] = x01
> MechaQMC5883_WriteReg(): ID[x0D] : [x09] = x1D
- cx=1428 , cy=-2300 , cz=-1063
ACC  : X=7166 , Y=1896 , Z=-16074
GYRO : X=-546 , Y=-105 , Z=-558

 Temperature:2792
 Id=FF > MechaQMC5883_WriteReg(): ID[x0D] : [x0B] = x01
> MechaQMC5883_WriteReg(): ID[x0D] : [x09] = x1D
- cx=1378 , cy=-2271 , cz=-1157
ACC  : X=6998 , Y=2022 , Z=-15828
GYRO : X=1327 , Y=5628 , Z=1786

 Temperature:2791
 Id=FF > MechaQMC5883_WriteReg(): ID[x0D] : [x0B] = x01
> MechaQMC5883_WriteReg(): ID[x0D] : [x09] = x1D
- cx=1412 , cy=-2300 , cz=-1037


*/

/*
 *@Note
 I2C interface routine to operate EEPROM peripheral:
 I2C1_SCL(PC2)\I2C1_SDA(PC1).
  This example uses EEPROM for AT24Cxx series.
  Steps:
 READ EEPROM:Start + 0xA0 + 8bit Data Address + Start + 0xA1 + Read Data + Stop.
 WRITE EERPOM:Start + 0xA0 + 8bit Data Address + Write Data + Stop.

*/

#include "debug.h"

///


u8   AT24CXX_ReadOneByte(  u16 ReadAddr ) ;
void AT24CXX_WriteOneByte( u16 WriteAddr, u8 DataToWrite ) ;
void AT24CXX_Read(         u16 ReadAddr, u8 *pBuffer, u16 NumToRead ) ;
void AT24CXX_Write(        u16 WriteAddr, u8 *pBuffer, u16 NumToWrite ) ;


///


#ifndef __MPU6050_H
#define __MPU6050_H


// #include "iic.h"
#include "debug.h"


//MPU6050 AD0
/*
#define MPU_AD0_H               GPIO_SetBits(  GPIOA,GPIO_Pin_15)    // SDA
#define MPU_AD0_L               GPIO_ResetBits(GPIOA,GPIO_Pin_15)  // SDA
*/
// 003-c8t6 : - SCL=A6 , SDA=A7 :
#define MPU_AD0_H               GPIO_SetBits(  GPIOC,GPIO_Pin_1)    // SDA
#define MPU_AD0_L               GPIO_ResetBits(GPIOC,GPIO_Pin_1)  // SDA
/*
// 203-c8t6 : - SCL=C2 , SDA=C1 :
#define MPU_AD0_H               GPIO_SetBits(  GPIOC,GPIO_Pin_1)    // SDA
#define MPU_AD0_L               GPIO_ResetBits(GPIOC,GPIO_Pin_1)  // SDA
*/


 #define MPU_ACCEL_OFFS_REG        0X06    //accel_offs
 #define MPU_PROD_ID_REG           0X0C    //prod id

#define MPU_SELF_TESTX_REG      0X0D    //X
#define MPU_SELF_TESTY_REG      0X0E    //Y
#define MPU_SELF_TESTZ_REG      0X0F    //Z
#define MPU_SELF_TESTA_REG      0X10    //A
#define MPU_SAMPLE_RATE_REG     0X19    //
#define MPU_CFG_REG             0X1A    //
#define MPU_GYRO_CFG_REG        0X1B    //
#define MPU_ACCEL_CFG_REG       0X1C    //
#define MPU_MOTION_DET_REG      0X1F    //
#define MPU_FIFO_EN_REG         0X23    //FIFO
#define MPU_I2CMST_CTRL_REG     0X24    //IIC
#define MPU_I2CSLV0_ADDR_REG    0X25    //IIC
#define MPU_I2CSLV0_REG         0X26    //IIC
#define MPU_I2CSLV0_CTRL_REG    0X27    //IIC
#define MPU_I2CSLV1_ADDR_REG    0X28    //IIC
#define MPU_I2CSLV1_REG         0X29    //IIC
#define MPU_I2CSLV1_CTRL_REG    0X2A    //IIC
#define MPU_I2CSLV2_ADDR_REG    0X2B    //IIC2
#define MPU_I2CSLV2_REG         0X2C    //IIC2
#define MPU_I2CSLV2_CTRL_REG    0X2D    //IIC2
#define MPU_I2CSLV3_ADDR_REG    0X2E    //IIC3
#define MPU_I2CSLV3_REG         0X2F    //IIC3
#define MPU_I2CSLV3_CTRL_REG    0X30    //IIC3
#define MPU_I2CSLV4_ADDR_REG    0X31    //IIC4
#define MPU_I2CSLV4_REG         0X32    //IIC4
#define MPU_I2CSLV4_DO_REG      0X33    //IIC4
#define MPU_I2CSLV4_CTRL_REG    0X34    //IIC4
#define MPU_I2CSLV4_DI_REG      0X35    //IIC4

#define MPU_I2CMST_STA_REG      0X36    //IIC
#define MPU_INTBP_CFG_REG       0X37    // /
#define MPU_INT_EN_REG          0X38    //
#define MPU_INT_STA_REG         0X3A    //

#define MPU_ACCEL_XOUTH_REG     0X3B    //X 8
#define MPU_ACCEL_XOUTL_REG     0X3C    //X 8
#define MPU_ACCEL_YOUTH_REG     0X3D    //Y 8
#define MPU_ACCEL_YOUTL_REG     0X3E    //Y 8
#define MPU_ACCEL_ZOUTH_REG     0X3F    //Z 8
#define MPU_ACCEL_ZOUTL_REG     0X40    //Z 8

#define MPU_TEMP_OUTH_REG       0X41    //
#define MPU_TEMP_OUTL_REG       0X42    //8

#define MPU_GYRO_XOUTH_REG      0X43    //X 8
#define MPU_GYRO_XOUTL_REG      0X44    //X 8
#define MPU_GYRO_YOUTH_REG      0X45    //Y 8
#define MPU_GYRO_YOUTL_REG      0X46    //Y 8
#define MPU_GYRO_ZOUTH_REG      0X47    //Z 8
#define MPU_GYRO_ZOUTL_REG      0X48    //Z 8

#define MPU_I2CSLV0_DO_REG      0X63    //IIC0
#define MPU_I2CSLV1_DO_REG      0X64    //IIC1
#define MPU_I2CSLV2_DO_REG      0X65    //IIC2
#define MPU_I2CSLV3_DO_REG      0X66    //IIC3

#define MPU_I2CMST_DELAY_REG    0X67    //IIC
#define MPU_SIGPATH_RST_REG     0X68    //
#define MPU_MDETECT_CTRL_REG    0X69    //
#define MPU_USER_CTRL_REG       0X6A    //
#define MPU_PWR_MGMT1_REG       0X6B    //1
#define MPU_PWR_MGMT2_REG       0X6C    //2
#define MPU_FIFO_CNTH_REG       0X72    //FIFO
#define MPU_FIFO_CNTL_REG       0X73    //FIFO
#define MPU_FIFO_RW_REG         0X74    //FIFO
#define MPU_DEVICE_ID_REG       0X75    //ID

#define MPU_ADDR                0X68

//#define MPU_READ    0XD1

//#define MPU_WRITE   0XD0

u8 MPU_Init(void);                              //MPU6050
u8 MPU_Write_Len(u8 addr,u8 reg,u8 len,u8 *buf);//IIC
u8 MPU_Read_Len(u8 addr,u8 reg,u8 len,u8 *buf); //IIC
u8 MPU_Write_Byte(u8 reg,u8 data);              //IIC
u8 MPU_Read_Byte(u8 reg);                       //IIC

u8 MPU_Set_Gyro_Fsr(u8 fsr);
u8 MPU_Set_Accel_Fsr(u8 fsr);
u8 MPU_Set_LPF(u16 lpf);
u8 MPU_Set_Rate(u16 rate);
u8 MPU_Set_Fifo(u8 sens);

short MPU_Get_Temperature(void);
u8 MPU_Get_Gyroscope(short *gx,short *gy,short *gz);
u8 MPU_Get_Accelerometer(short *ax,short *ay,short *az);


#endif

///

#define    HMC5883_ADDR     (0x1E)    // HMC5883L
#define    QMC5883_ADDR     (0x0D)    // QMC5883L

char  I2C_CHIPID = 0xFF ;

///

// MPU6050.c //

// #include "mpu6050.h"
/*
#include "debug.h"
*/

//

#ifdef OLD_CODE

/*
void  IIC_Start( u8 Flg )
{
    while( I2C_GetFlagStatus( I2C1, I2C_FLAG_BUSY ) != RESET );
    I2C_GenerateSTART( I2C1, Flg );

    while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_MODE_SELECT ) );
}
*/

void  IIC_Start()
{
    while( I2C_GetFlagStatus( I2C1, I2C_FLAG_BUSY ) != RESET );
    I2C_GenerateSTART( I2C1, ENABLE );

    while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_MODE_SELECT ) );
}

/*
void  IIC_Stop( u8 Flg )
{
    I2C_GenerateSTOP( I2C1, Flg );
}
*/

void  IIC_Stop()
{
    I2C_GenerateSTOP( I2C1, ENABLE );
}


int  IIC_WaitAck()
{
    while( I2C_GetFlagStatus( I2C1, I2C_FLAG_BUSY ) != RESET ) {}
return 0;
}


void  IIC_SendByte( u8 Dat )
{
    I2C_SendData( I2C1, Dat );
}

u8  IIC_ReadByte( u8 Dat )
{
u8  R ;
    R = I2C_ReceiveData( I2C1 );
return( R );
}

#endif  // - OLD_CODE .


/*

u8 MPU_Init(void)
{
    u8 res;
    GPIO_InitTypeDef  GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);     //AFIO
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);    //IO PORTA

//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;              //
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;              //
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;        //
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;       //IO 50MHz
    GPIO_Init(GPIOA, &GPIO_InitStructure);                  //GPIOA

    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE); //JTAG PA15 IO PA15 O!!!

    MPU_AD0_L;                              // MPU6050 AD0 :0X68

    IIC_Init();                             //IIC

    MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X80); //MPU6050
    Delay_Ms(100);
    MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X00); //MPU6050
    MPU_Set_Gyro_Fsr(3);                    //2000dps
    MPU_Set_Accel_Fsr(0);                   //2g
    MPU_Set_Rate(50);                       //50Hz
    MPU_Write_Byte(MPU_INT_EN_REG,0X00);    //
    MPU_Write_Byte(MPU_USER_CTRL_REG,0X00); //I2C
    MPU_Write_Byte(MPU_FIFO_EN_REG,0X00);   //FIFO
    MPU_Write_Byte(MPU_INTBP_CFG_REG,0X80); //INT

    res=MPU_Read_Byte(MPU_DEVICE_ID_REG);
    if(res==MPU_ADDR)// ID
    {
        MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X01); //CLKSEL,PLL X
        MPU_Write_Byte(MPU_PWR_MGMT2_REG,0X00); //
        MPU_Set_Rate(50);                       //50Hz
    }
    else
      return 1;
    return 0;
}

*/

/*

void  IIC_Init(u32 bound, u16 address)
// u8  MPU_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure={0};
    I2C_InitTypeDef  I2C_InitTSturcture={0};

    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE );
    RCC_APB1PeriphClockCmd( RCC_APB1Periph_I2C1, ENABLE );

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOC, &GPIO_InitStructure );

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOC, &GPIO_InitStructure );

    I2C_InitTSturcture.I2C_Mode        = I2C_Mode_I2C;
    I2C_InitTSturcture.I2C_ClockSpeed  = bound;
    I2C_InitTSturcture.I2C_DutyCycle   = I2C_DutyCycle_2;
    I2C_InitTSturcture.I2C_OwnAddress1 = address;
    I2C_InitTSturcture.I2C_Ack         = I2C_Ack_Enable;
    I2C_InitTSturcture.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;

    MPU_AD0_L ;                              // MPU6050 AD0 0X68

    I2C_Init( I2C1, &I2C_InitTSturcture );

    I2C_Cmd( I2C1, ENABLE );
//    IIC_SendByte( ENABLE ) ;  // - ???

//    I2C_AcknowledgeConfig( I2C1, ENABLE );
//    IIC_WaitAck();     // - ???
    while( I2C_GetFlagStatus( I2C1, I2C_FLAG_BUSY ) != RESET ) {} // - ???

    // ??? :

    MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X80); //MPU6050
    Delay_Ms(100);
    MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X00); //MPU6050

    MPU_Set_Gyro_Fsr(3);                    //+-2000dps
    MPU_Set_Accel_Fsr(0);                   //+-2g
    MPU_Set_Rate(50);                       //50Hz

    MPU_Write_Byte(MPU_INT_EN_REG,0X00);    //
    MPU_Write_Byte(MPU_USER_CTRL_REG,0X00); //I2C
    MPU_Write_Byte(MPU_FIFO_EN_REG,0X00);   //FIFO
    MPU_Write_Byte(MPU_INTBP_CFG_REG,0X80); //INT

int  res ;
    res = MPU_Read_Byte(MPU_DEVICE_ID_REG);
    if(res == MPU_ADDR)  // ID
    {
        MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X01); // CLKSEL,PLL X
        MPU_Write_Byte(MPU_PWR_MGMT2_REG,0X00); //
        MPU_Set_Rate(50);                       // 50Hz
    }
//    else
//      return 1;
//    return 0;

}

*/


//fsr:0,250dps;1,500dps;2,1000dps;3,2000dps
u8 MPU_Set_Gyro_Fsr(u8 fsr)
{
// //    return( MPU_Write_Byte(       MPU_GYRO_CFG_REG , fsr << 3 ) );//
            AT24CXX_WriteOneByte( MPU_GYRO_CFG_REG , fsr << 3 )  ;

return( 0 );
}

u8 MPU_Set_Accel_Fsr(u8 fsr)
{
// // return MPU_Write_Byte(       MPU_ACCEL_CFG_REG , fsr << 3 );//
       AT24CXX_WriteOneByte( MPU_ACCEL_CFG_REG , fsr << 3 )  ;

return( 0 );
}


u8 MPU_Set_LPF(u16 lpf)
{
u8 data = 0;

    if(      lpf >= 188 )
        data=1;
    else if( lpf >= 98 )
        data=2;
    else if( lpf >= 42 )
        data=3;
    else if( lpf >= 20 )
        data=4;
    else if( lpf >= 10 )
        data=5;
    else
        data = 6;

// // return( MPU_Write_Byte(       MPU_CFG_REG , data ) );
// return( AT24CXX_WriteOneByte( MPU_CFG_REG , data ) );
    AT24CXX_WriteOneByte( MPU_CFG_REG , data )  ;

return(0);
}


u8 MPU_Set_Rate(u16 rate)
{
u8 data;

    if(rate > 1000)
        rate = 1000;
    if(rate < 4)
        rate = 4;
    data = 1000 / rate - 1 ;

// //    data = MPU_Write_Byte(       MPU_SAMPLE_RATE_REG , data );  //
           AT24CXX_WriteOneByte( MPU_SAMPLE_RATE_REG , data );    //

return MPU_Set_LPF( rate / 2 ); // LPF
}


short MPU_Get_Temperature(void)
{
u8 buf[2];
short raw;
float temp;

// //    MPU_Read_Len( MPU_ADDR , MPU_TEMP_OUTH_REG , 2 , buf );
    AT24CXX_Read(            MPU_TEMP_OUTH_REG , buf , 2 );

    raw  = ((u16)buf[0]<<8) | buf[1];
    temp = 36.53 + ((double)raw) / 340;

return( temp * 100 );
}


u8 MPU_Get_Gyroscope(short *gx,short *gy,short *gz)
{
u8 buf[6],res = 0 ;

// //    res = MPU_Read_Len( MPU_ADDR , MPU_GYRO_XOUTH_REG , 6 , buf );
         AT24CXX_Read(            MPU_GYRO_XOUTH_REG , buf , 6 );

// //    if( res == 0 )
    {
        *gx = ((u16)buf[0]<<8) | buf[1];
        *gy = ((u16)buf[2]<<8) | buf[3];
        *gz = ((u16)buf[4]<<8) | buf[5];
        printf("GYRO : X=%d , Y=%d , Z=%d  \r\n",*gx,*gy,*gz);
    }

return res;;
}


u8 MPU_Get_Accelerometer(short *ax,short *ay,short *az)
{
u8 buf[6] , res = 0 ;

// //    res = MPU_Read_Len( MPU_ADDR , MPU_ACCEL_XOUTH_REG , 6 , buf );
          AT24CXX_Read(            MPU_ACCEL_XOUTH_REG , buf , 6 );

// //    if( res == 0 )
    {
        *ax = ((u16)buf[0]<<8) | buf[1];
        *ay = ((u16)buf[2]<<8) | buf[3];
        *az = ((u16)buf[4]<<8) | buf[5];

        printf("ACC  : X=%d , Y=%d , Z=%d  \r\n", *ax , *ay , *az );
    }

return res;;
}


//

int  cX,cY,cZ; //triple axis data
int  Id ;


//


 // #define  HMC5883_E  1

 // #define    HMC5883_ADDR     (0x1E)    // HMC5883L
// // #define    QMC5883_ADDR     (0x0D)    // QMC5883L

// int cX,cY,cZ; //triple axis data
// int  Id ;

void  HMC_Get_Compass()
{

#ifdef  HMC5883_E

// int cX,cY,cZ; //triple axis data
// int  Id ;

//   CHIP_ADDR = HMC5883_ADDR_W ;
// //    DeviceSet( HMC5883_ADDR );
 //    DeviceSet( HMC5883_ADDR << 1 );
  I2C_CHIPID = ( HMC5883_ADDR ) ;

 // #define SNOOP_DVC_E  1
#ifdef  SNOOP_DVC_E
 printf(": I2C Snoop : \r\n");
  for(int i=0x01; i<=0xFF; i+=2)
  {
    CHIP_ADDR = ( HMC5883_ADDR ) ;
// //    printf( "* %02X  ," , readRegister(i)    );
    printf( "* %02X  ," , AT24CXX_ReadOneByte(i)    );
  }
 printf("\r\n");
#endif


 // CHIP_ADDR = HMC5883_ADDR_W ;
// //  DeviceSet( HMC5883_ADDR );
 //  DeviceSet( HMC5883_ADDR << 1 );

// //  Id = readRegister(0x75);
  Id = AT24CXX_ReadOneByte(0x75);
 printf( " Id=%0X "   , Id );

// //  writeRegister(0x02,0x00); //select mode register - continuous measurement mode
  AT24CXX_WriteOneByte(0x02,0X00); //

 // #define PRT_REGS_3  1
#ifdef  PRT_REGS_3
// //  printf( "%02X" ,  readRegister(0x03) );
// //  printf( "%02X " ,  readRegister(0x04) );
// //  printf( "%02X" ,  readRegister(0x05) );
// //  printf( "%02X " ,  readRegister(0x06) );
// //  printf( "%02X" ,  readRegister(0x07) );
// //  printf( "%02X " ,  readRegister(0x08) );
  printf( "%02X" ,  AT24CXX_ReadOneByte(0x03) );
  printf( "%02X " ,  AT24CXX_ReadOneByte(0x04) );
  printf( "%02X" ,  AT24CXX_ReadOneByte(0x05) );
  printf( "%02X " ,  AT24CXX_ReadOneByte(0x06) );
  printf( "%02X" ,  AT24CXX_ReadOneByte(0x07) );
  printf( "%02X " ,  AT24CXX_ReadOneByte(0x08) );
#endif

// //  cX = readRegister(0x03)<<8 | readRegister(0x04) ;
// //  cZ = readRegister(0x05)<<8 | readRegister(0x06) ;
// //  cY = readRegister(0x07)<<8 | readRegister(0x08) ;
  cX = AT24CXX_ReadOneByte(0x03)<<8 | AT24CXX_ReadOneByte(0x04) ;
  cZ = AT24CXX_ReadOneByte(0x05)<<8 | AT24CXX_ReadOneByte(0x06) ;
  cY = AT24CXX_ReadOneByte(0x07)<<8 | AT24CXX_ReadOneByte(0x08) ;

// //  cX = ( (cX >= 0x8000) ? (cX = cX - 0x10000) : (cX = cX) );
  cX = ( (cX >= 0x8000) ? (cX = cX - 0x10000) : (cX) );
// //  cY = ( (cY >= 0x8000) ? (cY = cY - 0x10000) : (cY = cY) );
  cY = ( (cY >= 0x8000) ? (cY = cY - 0x10000) : (cY) );
// //  cZ = ( (cZ >= 0x8000) ? (cZ = cZ - 0x10000) : (cZ = cZ) );
  cZ = ( (cZ >= 0x8000) ? (cZ = cZ - 0x10000) : (cZ) );

 // #define PRT_REGS_4  1
#ifdef  PRT_REGS_4
 printf( "  cX=%04X  " , cX & 0xFFFF );
 printf( "cY=%04X  "   , cY & 0xFFFF );
 printf( "cZ=%04X  "   , cZ & 0xFFFF );
// printf( "\r\n" );
#endif

 printf( "  cX=%d  " , cX  );
 printf( "cY=%d  "   , cY  );
 printf( "cZ=%d  "   , cZ  );
 printf( "\r\n" );

#endif  // - HMC5883_E .

}


///


  #define  QMC5883_E  1

#ifdef  QMC5883_E


/*

Arduino
#include <Wire.h>
#include <MechaQMC5883.h>

MechaQMC5883 qmc;

void setup()
{
  Wire.begin();
  Serial.begin(9600);
  qmc.init();
  //qmc.setMode(Mode_Continuous,ODR_200Hz,RNG_2G,OSR_256);
}

void loop()
{
int  x,y,z;
  qmc.read(&x,&y,&z);

Serial.print("x: ");
Serial.print(x);
Serial.print(" y: ");
Serial.print(y);
Serial.print(" z: ");
Serial.print(z);
Serial.println();
delay(100);
}

*/

//


// // #include "MechaQMC5883.h"

//

#ifndef  Mecha_QMC5883
#define  Mecha_QMC5883

// // #include "Arduino.h"
// // #include "Wire.h"

// #define QMC5883_ADDR  0x0D

//REG CONTROL

//0x09

#define Mode_Standby    0b00000000
#define Mode_Continuous 0b00000001

#define ODR_10Hz        0b00000000
#define ODR_50Hz        0b00000100
#define ODR_100Hz       0b00001000
#define ODR_200Hz       0b00001100

#define RNG_2G          0b00000000
#define RNG_8G          0b00010000

#define OSR_512         0b00000000
#define OSR_256         0b01000000
#define OSR_128         0b10000000
#define OSR_64          0b11000000


// // class MechaQMC5883
// // {
// // public:

void  MechaQMC5883_setAddress(uint8_t addr);

void  MechaQMC5883_init(); //init qmc5883

void  MechaQMC5883_setMode(uint16_t mode,uint16_t odr,uint16_t rng,uint16_t osr); // setting

void  MechaQMC5883_softReset(); //soft RESET

int  MechaQMC5883_read_iii( int* x,int* y,int* z); //reading
int  MechaQMC5883_read_iiii(int* x,int* y,int* z,int* a);
int  MechaQMC5883_read_iiif( int* x,int* y,int* z,float* a);

float  MechaQMC5883_azimuth_F(int* a,int* b);

// // private:

float  PI = 3.14159265359 ;

// void  MechaQMC5883_WriteReg(uint8_t Reg,uint8_t val);
void MechaQMC5883_WriteReg(char Reg, char val) ;

uint8_t  MechaQMC5883_address = QMC5883_ADDR;

// // };

#endif

//


// // #include <Wire.h>

// // void MechaQMC5883::setAddress(uint8_t addr){
void MechaQMC5883_setAddress(uint8_t addr)
{
    /// ChipID : ///
    MechaQMC5883_address = addr;
}

// // void MechaQMC5883::WriteReg(byte Reg,byte val){
void  MechaQMC5883_init()
{
    MechaQMC5883_WriteReg(0x0B,0x01);
  //Define Set/Reset period
// //  setMode(Mode_Continuous,ODR_200Hz,RNG_8G,OSR_512);
  MechaQMC5883_setMode( Mode_Continuous , ODR_200Hz , RNG_8G , OSR_512 );
/*
  Define
  OSR = 512
  Full Scale Range = 8G(Gauss)
  ODR = 200HZ
  set continuous measurement mode
*/
}

void MechaQMC5883_WriteReg(char Reg, char val)
{

// //  Wire.beginTransmission(address); //start talking
    I2C_CHIPID = ( QMC5883_ADDR ) ;
//    I2C_CHIPID = ( MechaQMC5883_address ) ;

// printf("> MechaQMC5883_WriteReg(): ID[x%02X] : [x%02X] = x%02X \r\n" , I2C_CHIPID , Reg , val );

// //   Wire.write(Reg); // Tell the HMC5883 to Continuously Measure
// //   Wire.write(val); // Set the Register
// //  Wire.endTransmission();
  AT24CXX_WriteOneByte(Reg,val); //
}

// // void MechaQMC5883::init(){
// //void MechaQMC5883::setMode(uint16_t mode,uint16_t odr,uint16_t rng,uint16_t osr){
void  MechaQMC5883_setMode(uint16_t mode,uint16_t odr,uint16_t rng,uint16_t osr)
{
    MechaQMC5883_WriteReg( 0x09 , mode | odr | rng | osr );
}


// // void MechaQMC5883::softReset(){
void  MechaQMC5883_softReset()
{
    MechaQMC5883_WriteReg(0x0A,0x80);
}


/**
 * read values from device
 * @return status value:
 *  - 0:success
 *  - 1:data too long to fit in transmit buffer
 *  - 2:received NACK on transmit of address
 *  - 3:received NACK on transmit of data
 *  - 4:other error
 *  - 8:overflow (magnetic field too strong)
 */
// // int MechaQMC5883::read(int* x,int* y,int* z){
int  MechaQMC5883_read_iii(int* x,int* y,int* z)
{

// //  Wire.beginTransmission(address);
  I2C_CHIPID = ( QMC5883_ADDR ) ;
//    I2C_CHIPID = ( MechaQMC5883_address ) ;

/*

   while( I2C_GetFlagStatus( I2C1, I2C_FLAG_BUSY ) != RESET );
  I2C_GenerateSTART( I2C1, ENABLE );
   while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_MODE_SELECT ) );

// //  Wire.write(0x00);
    I2C_SendData( I2C1, (u8)(0x00) );  // - send Read Address
 //     while( ! I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED ) );
     while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED ) );
     while( I2C_GetFlagStatus( I2C1, I2C_FLAG_RXNE ) ==  RESET )
    I2C_AcknowledgeConfig( I2C1, DISABLE );

// // int err = Wire.endTransmission();
 ////  while( ! I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED ) );
 //  I2C_GenerateSTOP( I2C1, ENABLE );
 //   while( I2C_GetFlagStatus( I2C1, I2C_FLAG_BUSY ) != RESET );
// //  if (err)
// //  {
// // return err;
// //  }

*/

// //  Wire.requestFrom(address, 7);
// //  *x = (int)(int16_t)(Wire.read() | Wire.read() << 8);
// //  *y = (int)(int16_t)(Wire.read() | Wire.read() << 8);
// //  *z = (int)(int16_t)(Wire.read() | Wire.read() << 8);
// // byte  overflow = Wire.read() & 0x02;
//  *x = (int)(int16_t)(I2C_ReceiveData(I2C1) | I2C_ReceiveData(I2C1) << 8);
//  *y = (int)(int16_t)(I2C_ReceiveData(I2C1) | I2C_ReceiveData(I2C1) << 8);
//  *z = (int)(int16_t)(I2C_ReceiveData(I2C1) | I2C_ReceiveData(I2C1)) << 8);
// char  overflow =      I2C_ReceiveData(I2C1) & 0x02;
  *x = (int)(int16_t)(AT24CXX_ReadOneByte(0) | AT24CXX_ReadOneByte(1) << 8);
  *y = (int)(int16_t)(AT24CXX_ReadOneByte(2) | AT24CXX_ReadOneByte(3) << 8);
  *z = (int)(int16_t)(AT24CXX_ReadOneByte(4) | AT24CXX_ReadOneByte(5) << 8);
char  overflow =      AT24CXX_ReadOneByte(6) & 0x02;
//  *x = (int)(int16_t)(AT24CXX_ReadOneByte(MechaQMC5883_address+0) | AT24CXX_ReadOneByte(MechaQMC5883_address+1) << 8);
//  *y = (int)(int16_t)(AT24CXX_ReadOneByte(MechaQMC5883_address+2) | AT24CXX_ReadOneByte(MechaQMC5883_address+3) << 8);
//  *z = (int)(int16_t)(AT24CXX_ReadOneByte(MechaQMC5883_address+4) | AT24CXX_ReadOneByte(MechaQMC5883_address+5) << 8);
// char  overflow =      AT24CXX_ReadOneByte(MechaQMC5883_address+6) & 0x02;

return( overflow << 2 );
}

// // int MechaQMC5883::read(int* x,int* y,int* z,int* a){
int MechaQMC5883_read_iiii(int* x,int* y,int* z,int* a)
{
int err = MechaQMC5883_read_iii(x,y,z);
  *a = MechaQMC5883_azimuth_F(y,x);
return err;
}

// // int MechaQMC5883::read(int* x,int* y,int* z,float* a){
int MechaQMC5883_read_iiif(int* x,int* y,int* z,float* a)
{
int err = MechaQMC5883_read_iii(x,y,z);
  *a = MechaQMC5883_azimuth_F(y,x);
return err;
}


// // float MechaQMC5883::azimuth(int *a, int *b){
float MechaQMC5883_azimuth_F(int *a, int *b)
{

    // // float  MechaQMC5883_azimuth_V = atan2((int)*a,(int)*b) * 180.0 / PI;
// // return( (MechaQMC5883_azimuth_V < 0) ? (360 + MechaQMC5883_azimuth_V) : MechaQMC5883_azimuth_V );

return(0);
}


#endif  // - QMC5883_E .


///

//  #define  QMC5883_E  1

// // #define    HMC5883_ADDR     (0x1E)    // HMC5883L
 // #define    QMC5883_ADDR     (0x0D)    // QMC5883L

// int cX,cY,cZ; //triple axis data
// int  Id ;

void  QMC_Get_Compass()
{

#ifdef  QMC5883_E

// int  cX,cY,cZ; //triple axis data
// int  Id ;

//   CHIP_ADDR = HMC5883_ADDR_W ;
// //    DeviceSet( HMC5883_ADDR );
 //    DeviceSet( HMC5883_ADDR << 1 );

 // #define SNOOP_DVC_E  1
#ifdef  SNOOP_DVC_E
 printf(": I2C Snoop : \r\n");
  for(int i=0x01; i<=0xFF; i+=2)
  {
    CHIP_ADDR = ( QMC5883_ADDR ) ;
// //    printf( "* %02X  ," , readRegister(i)    );
    printf( "* %02X  ," , AT24CXX_ReadOneByte(i)    );
  }
 printf("\r\n");
#endif

 printf( "CPS  :  "   , Id );

 // CHIP_ADDR = QMC5883_ADDR_W ;
// //  DeviceSet( QMC5883_ADDR );
 //  DeviceSet( QMC5883_ADDR << 1 );
  I2C_CHIPID = ( QMC5883_ADDR ) ;

// //  Id = readRegister(0x75);
  Id = AT24CXX_ReadOneByte(0x75);
 printf( "Id=%0X "   , Id );

 //
  MechaQMC5883_init();
  //qmc.setMode(Mode_Continuous,ODR_200Hz,RNG_2G,OSR_256);
 //

/*

// //  writeRegister(0x02,0x00); //select mode register - continuous measurement mode
  AT24CXX_WriteOneByte(0x02,0X00); //

 // #define PRT_REGS_3  1
#ifdef  PRT_REGS_3
// //  printf( "%02X" ,  readRegister(0x03) );
// //  printf( "%02X " ,  readRegister(0x04) );
// //  printf( "%02X" ,  readRegister(0x05) );
// //  printf( "%02X " ,  readRegister(0x06) );
// //  printf( "%02X" ,  readRegister(0x07) );
// //  printf( "%02X " ,  readRegister(0x08) );
  printf( "%02X" ,  AT24CXX_ReadOneByte(0x03) );
  printf( "%02X " ,  AT24CXX_ReadOneByte(0x04) );
  printf( "%02X" ,  AT24CXX_ReadOneByte(0x05) );
  printf( "%02X " ,  AT24CXX_ReadOneByte(0x06) );
  printf( "%02X" ,  AT24CXX_ReadOneByte(0x07) );
  printf( "%02X " ,  AT24CXX_ReadOneByte(0x08) );
#endif

// //  cX = readRegister(0x03)<<8 | readRegister(0x04) ;
// //  cZ = readRegister(0x05)<<8 | readRegister(0x06) ;
// //  cY = readRegister(0x07)<<8 | readRegister(0x08) ;
  cX = AT24CXX_ReadOneByte(0x03)<<8 | AT24CXX_ReadOneByte(0x04) ;
  cZ = AT24CXX_ReadOneByte(0x05)<<8 | AT24CXX_ReadOneByte(0x06) ;
  cY = AT24CXX_ReadOneByte(0x07)<<8 | AT24CXX_ReadOneByte(0x08) ;

// //  cX = ( (cX >= 0x8000) ? (cX = cX - 0x10000) : (cX = cX) );
  cX = ( (cX >= 0x8000) ? (cX = cX - 0x10000) : (cX) );
// //  cY = ( (cY >= 0x8000) ? (cY = cY - 0x10000) : (cY = cY) );
  cY = ( (cY >= 0x8000) ? (cY = cY - 0x10000) : (cY) );
// //  cZ = ( (cZ >= 0x8000) ? (cZ = cZ - 0x10000) : (cZ = cZ) );
  cZ = ( (cZ >= 0x8000) ? (cZ = cZ - 0x10000) : (cZ) );

 // #define PRT_REGS_4  1
#ifdef  PRT_REGS_4
 printf( "  cX=%04X  " , cX & 0xFFFF );
 printf( "cY=%04X  "   , cY & 0xFFFF );
 printf( "cZ=%04X  "   , cZ & 0xFFFF );
// printf( "\r\n" );
#endif

 printf( "  cX=%d  " , cX  );
 printf( "cY=%d  "   , cY  );
 printf( "cZ=%d  "   , cZ  );
 printf( "\r\n" );
#endif  // - QMC5883_E .

*/

int x,y,z;
  MechaQMC5883_read_iii(&x,&y,&z);
  cX = x;  cY = y ;  cZ = z ;

  printf("cx=%d , cy=%d , cz=%d \r\n" , x , y , z );

   Delay_Ms(100);

#endif // - QMC5883_E .

}


//


#ifdef  OLD_CODE

u8 MPU_Write_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{
    u8 i;
    IIC_Start();
    IIC_SendByte((addr<<1)|0);  //
    if( IIC_WaitAck() )           //
    {
        IIC_Stop();
        return 1;
    }

    IIC_SendByte(reg); //
    IIC_WaitAck();     //

    for(i=0;i<len;i++)
    {
        IIC_SendByte(buf[i]);  //
        if( IIC_WaitAck() )      //ACK
        {
            IIC_Stop();
            return 1;
        }
    }
    IIC_Stop();
    return 0;
}

u8 MPU_Read_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{
    IIC_Start();
    IIC_SendByte((addr<<1)|0);//
    if(IIC_WaitAck())  //
    {
        IIC_Stop();
        return 1;
    }
    IIC_SendByte(reg); //
    IIC_WaitAck();     //

    IIC_Start();
    IIC_SendByte((addr<<1)|1);//
    IIC_WaitAck();     //
    while(len)
    {
        if(len==1)
            * buf = IIC_ReadByte(0);//nACK
        else
            * buf = IIC_ReadByte(1);     //ACK
printf( " <%02X> " , * buf );
        len--;
        buf++;
    }
    IIC_Stop(); // STOP
    return 0;
}


u8 MPU_Write_Byte(u8 reg,u8 data)
{
  IIC_Start();
    IIC_SendByte((MPU_ADDR<<1)|0);//
    if(IIC_WaitAck())  //
    {
        IIC_Stop();
return 1;
    }
    IIC_SendByte(reg); //
    IIC_WaitAck();     //

    IIC_SendByte(data);//
    if(IIC_WaitAck())  //ACK
    {
        IIC_Stop();
return 1;
    }
  IIC_Stop();

return 0;
}


u8 MPU_Read_Byte(u8 reg)
{
u8 res;

    IIC_Start();
     IIC_SendByte((MPU_ADDR<<1)|0);//
      IIC_WaitAck();     //
     IIC_SendByte(reg); //
      IIC_WaitAck();     //

    IIC_Start();
     IIC_SendByte((MPU_ADDR<<1)|1);//
      IIC_WaitAck();     //
     res=IIC_ReadByte(0);//nACK

    IIC_Stop();         // STOP

printf( " <%02X> " , res );

return res;
}

#endif  // - OLD_CODE


// int main(void)
// int main_1(void)
void  MPU6050_TEST(void)
{
short aacx,aacy,aacz;       //
short gyrox,gyroy,gyroz;    //
short temp;                 //

/*

        NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
        Delay_Init();
        USART_Printf_Init(115200);
//        MPU_Init();                //MPU6050
//        IIC_Init( 100000, 0xA0 );
        IIC_Init( 100000, 0x68 );
//        IIC_Init( 100000, 0x69 );

        printf("\r\n\r\n: MPU6050 TEST : \r\n\r\n");
        printf("- SystemClk : %d \r\n\r\n\r\n",SystemCoreClock);

*/

//        while( 1 )
//        {
            temp = MPU_Get_Temperature(); //
            MPU_Get_Accelerometer(&aacx,&aacy,&aacz);   //
            MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);    //

//            HMC_Get_Compass();

//            printf("\r\n Temperature:%d \r\n",temp);
            printf("Temp : %d \r\n\r\n" , temp );

//             Delay_Ms(500);

//        }

}

///


/**********************************************************************
*@Note:
  AT24Cxx
    READ EEPROM Start + 0xA0 + 8bit Data Address + Start + 0xA1 + Read Data + Stop.
    WRITE EERPOM Start + 0xA0 + 8bit Data Address + Write Data + Stop.
*******************************************************************************/
/* EERPOM DATA ADDRESS Length Definition */
#define Address_8bit  0
#define Address_16bit  1

/* EERPOM DATA ADDRESS Length Selection */
#define Address_Lenth   Address_8bit
//#define Address_Lenth   Address_16bit

/* Global define */
#define SIZE sizeof(TEXT_Buffer)

/* Global Variable */
const u8 TEXT_Buffer[]={"CH32V00x I2C TEST"};

/*********************************************************************
 * @fn      IIC_Init
 * @brief   Initializes the IIC peripheral.
 * @return  none
 */
void IIC_Init(u32 bound, u16 address)
{
printf("> IIC_Init() : bound=%d , address=%02X :\r\n" , bound , address );

    GPIO_InitTypeDef GPIO_InitStructure = {0};
    I2C_InitTypeDef  I2C_InitTSturcture = {0};

    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE );
    RCC_APB1PeriphClockCmd( RCC_APB1Periph_I2C1, ENABLE );

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOC, &GPIO_InitStructure );

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOC, &GPIO_InitStructure );

    I2C_InitTSturcture.I2C_Mode                = I2C_Mode_I2C;
    I2C_InitTSturcture.I2C_OwnAddress1         = address;
    I2C_InitTSturcture.I2C_ClockSpeed          = bound;
    I2C_InitTSturcture.I2C_DutyCycle           = I2C_DutyCycle_2;
    I2C_InitTSturcture.I2C_Ack                 = I2C_Ack_Enable;
    I2C_InitTSturcture.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init( I2C1, &I2C_InitTSturcture );

    I2C_Cmd( I2C1, ENABLE );

    I2C_AcknowledgeConfig( I2C1, ENABLE );

printf("< IIC_Init() .\r\n");
}

/*********************************************************************
 * @fn      AT24CXX_Init
 * @brief   Initializes AT24xx EEPROM.
 * @return  none
 */
void AT24CXX_Init(void)
{
    IIC_Init( 100000, 0xA0 );
}

void MPU6050_Init(void)
{
    I2C_CHIPID = ( MPU_ADDR ) ;
    IIC_Init( 100000, 0x68 );
}

void HMC5883_Init(void)
{
    I2C_CHIPID = ( HMC5883_ADDR ) ;
    IIC_Init( 100000, 0x1E );
}

void QMC5883_Init(void)
{
    I2C_CHIPID = ( QMC5883_ADDR ) ;
    IIC_Init( 100000, 0x0D );
}


/*********************************************************************
 * @fn      AT24CXX_ReadOneByte
 * @brief   Read one data from EEPROM.
 * @param   ReadAddr - Read first address.
 * @return  temp - Read data.
 */
u8 AT24CXX_ReadOneByte(u16 ReadAddr)
{
    u8 temp=0;

    while( I2C_GetFlagStatus( I2C1, I2C_FLAG_BUSY ) != RESET );
    I2C_GenerateSTART( I2C1, ENABLE );

    while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_MODE_SELECT ) );
// //    I2C_Send7bitAddress( I2C1, 0XA0, I2C_Direction_Transmitter );
 //    I2C_Send7bitAddress( I2C1, 0X68 , I2C_Direction_Transmitter );
  //    I2C_Send7bitAddress( I2C1, (0X68<<1) , I2C_Direction_Transmitter );
    I2C_Send7bitAddress( I2C1, (I2C_CHIPID<<1) , I2C_Direction_Transmitter );

    while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED ) );


    // Send Address 8b / 16b : //

#if (Address_Lenth  == Address_8bit)
    I2C_SendData( I2C1, (u8)(ReadAddr&0x00FF) );
    while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED ) );

#elif (Address_Lenth  == Address_16bit)
    I2C_SendData( I2C1, (u8)(ReadAddr>>8) );
    while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED ) );

    I2C_SendData( I2C1, (u8)(ReadAddr&0x00FF) );
    while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED ) );

#endif


    I2C_GenerateSTART( I2C1, ENABLE );

     while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_MODE_SELECT ) );
// //    I2C_Send7bitAddress( I2C1, 0XA0, I2C_Direction_Receiver );
 //    I2C_Send7bitAddress( I2C1, 0X68 , I2C_Direction_Receiver );
  //    I2C_Send7bitAddress( I2C1, (0X68<<1) , I2C_Direction_Receiver );
    I2C_Send7bitAddress( I2C1, (I2C_CHIPID<<1) , I2C_Direction_Receiver );

     while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED ) );
     while( I2C_GetFlagStatus( I2C1, I2C_FLAG_RXNE ) ==  RESET )
    I2C_AcknowledgeConfig( I2C1, DISABLE );

    temp = I2C_ReceiveData( I2C1 );

    I2C_GenerateSTOP( I2C1, ENABLE );

    return temp;
}


/*********************************************************************
 * @fn      AT24CXX_WriteOneByte
 * @brief   Write one data to EEPROM.
 * @param   WriteAddr - Write frist address.
 * @return  DataToWrite - Write data.
 */
void AT24CXX_WriteOneByte(u16 WriteAddr, u8 DataToWrite)
{
    while( I2C_GetFlagStatus( I2C1, I2C_FLAG_BUSY ) != RESET );
    I2C_GenerateSTART( I2C1, ENABLE );

    while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_MODE_SELECT ) );
// //    I2C_Send7bitAddress( I2C1, 0XA0, I2C_Direction_Transmitter );
 //    I2C_Send7bitAddress( I2C1, 0X68 , I2C_Direction_Transmitter );
  //    I2C_Send7bitAddress( I2C1, (0X68<<1) , I2C_Direction_Transmitter );
    I2C_Send7bitAddress( I2C1, (I2C_CHIPID<<1) , I2C_Direction_Transmitter );

    while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED ) );


    // Send Address 8b / 16b : //

#if (Address_Lenth  == Address_8bit)
    I2C_SendData( I2C1, (u8)(WriteAddr&0x00FF) );
    while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED ) );

#elif (Address_Lenth  == Address_16bit)
    I2C_SendData( I2C1, (u8)(WriteAddr>>8) );
    while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED ) );

    I2C_SendData( I2C1, (u8)(WriteAddr&0x00FF) );
    while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED ) );

#endif


    if( I2C_GetFlagStatus( I2C1, I2C_FLAG_TXE ) !=  RESET )
    {
        I2C_SendData( I2C1, DataToWrite );
    }

    while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED ) );
    I2C_GenerateSTOP( I2C1, ENABLE );
}


/*********************************************************************
 * @fn      AT24CXX_Read
 * @brief   Read multiple data from EEPROM.
 * @param   ReadAddr - Read frist address. (AT24c02: 0~255)
 *          pBuffer - Read data.
 *          NumToRead - Data number.
 * @return  none
 */
void AT24CXX_Read(u16 ReadAddr, u8 *pBuffer, u16 NumToRead)
{
    while( NumToRead )
    {
        * pBuffer ++ = AT24CXX_ReadOneByte( ReadAddr ++ );
         NumToRead -- ;
    }
}


/*********************************************************************
 * @fn      AT24CXX_Write
 * @brief   Write multiple data to EEPROM.
 * @param   WriteAddr - Write frist address. (AT24c02: 0~255)
 *          pBuffer - Write data.
 *          NumToWrite - Data number.
 * @return  none
 */
void AT24CXX_Write(u16 WriteAddr, u8 *pBuffer, u16 NumToWrite)
{
    while(NumToWrite--)
    {
        AT24CXX_WriteOneByte( WriteAddr , * pBuffer );
         WriteAddr ++ ;
         pBuffer ++ ;
          Delay_Ms(2);
    }
}


// // void MPU6050_Write(u16 WriteAddr, u8 *pBuffer, u16 NumToWrite)
void MPU6050_Write()
{
printf("> MPU6050_Write() : \r\n");

// //     MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X80); //MPU6050
     AT24CXX_WriteOneByte(MPU_PWR_MGMT1_REG,0X80); //MPU6050
      Delay_Ms(100);
// //     MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X00); //MPU6050
     AT24CXX_WriteOneByte(MPU_PWR_MGMT1_REG,0X00); //MPU6050
//      Delay_Ms(1);

printf("= MPU6050_Write() -> MPU_Set_Gyro_Fsr() \r\n");
// //     MPU_Set_Gyro_Fsr(3);                    //+-2000dps
     AT24CXX_WriteOneByte( MPU_ACCEL_CFG_REG , 3 << 3 )  ;
//      Delay_Ms(1);
printf("= MPU6050_Write() -> MPU_Set_Accel_Fsr() \r\n");
     MPU_Set_Accel_Fsr(0);                   //+-2g
printf("= MPU6050_Write() -> MPU_Set_Rate() \r\n");
     MPU_Set_Rate(50);                       //50Hz

printf("= MPU6050_Write() ... \r\n");

// //     MPU_Write_Byte(MPU_INT_EN_REG,0X00);    //
     AT24CXX_WriteOneByte(MPU_INT_EN_REG,0X00);    //
// //     MPU_Write_Byte(MPU_USER_CTRL_REG,0X00); //I2C
     AT24CXX_WriteOneByte(MPU_USER_CTRL_REG,0X00); //I2C
// //     MPU_Write_Byte(MPU_FIFO_EN_REG,0X00);   //FIFO
     AT24CXX_WriteOneByte(MPU_FIFO_EN_REG,0X00);   //FIFO
// //     MPU_Write_Byte(MPU_INTBP_CFG_REG,0X80); //INT
     AT24CXX_WriteOneByte(MPU_INTBP_CFG_REG,0X80); //INT

// // int  res ;
// //     res = MPU_Read_Byte(MPU_DEVICE_ID_REG);
     AT24CXX_ReadOneByte( MPU_DEVICE_ID_REG );
// //     if( res == MPU_ADDR )  // ID
     {
// //         MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X01); // CLKSEL,PLL X
         AT24CXX_WriteOneByte(MPU_PWR_MGMT1_REG,0X01); // CLKSEL,PLL X
// //         MPU_Write_Byte(MPU_PWR_MGMT2_REG,0X00); //
         AT24CXX_WriteOneByte(MPU_PWR_MGMT2_REG,0X00); //

         MPU_Set_Rate(50);                       // 50Hz
     }
 //    else
 //      return 1;
 //    return 0;

printf("< MPU6050_Write() . \r\n");
}


/*********************************************************************
 * @fn      main
 * @brief   Main program.
 * @return  none
 */
int main(void)
{
// u8 data[SIZE];

    Delay_Init();
    USART_Printf_Init(115200);
// //    printf(": eeprom 24Cxx :\r\n");
    printf("\r\n\r\n: MPU6050 003 :\r\n");
     printf("- SystemClk:%d\r\n",SystemCoreClock);

    //

/*
    printf("Start Write 24Cxx....\r\n");
     AT24CXX_Write(100,(u8*)TEXT_Buffer,SIZE);
    printf("24Cxx Write Success!\r\n");

    Delay_Ms(500);

    printf("Start Read 24Cxx....\r\n");
     AT24CXX_Read(100,data,SIZE);
    printf("The Data Read Is: \r\n");
     printf("%s\r\n", data);
*/


     // //    AT24CXX_Init();

     MPU6050_Init();
     I2C_CHIPID = ( MPU_ADDR ) ;
//     I2C_CHIPID = ( MPU_ADDR << 1 ) ;

     //

#ifdef  HMC5883_E
printf(": HMC5883 :\r\n");
     //   CHIP_ADDR = HMC5883_ADDR_W ;
// //     DeviceSet( HMC5883_ADDR );
 //     DeviceSet( HMC5883_ADDR << 1 );
    I2C_CHIPID = ( HMC5883_ADDR ) ;
//    I2C_CHIPID = ( HMC5883_ADDR << 1 ) ;

// //     writeRegister(0x02,0x00); //select mode register - continuous measurement mode
     AT24CXX_WriteOneByte(0x02,0X00); //
#endif

         //

#ifdef  QMC5883_E
printf(": QMC5883 :\r\n");
     //   CHIP_ADDR = QMC5883_ADDR_W ;
// //     DeviceSet( QMC5883_ADDR );
 //     DeviceSet( QMC5883_ADDR << 1 );
    I2C_CHIPID = ( QMC5883_ADDR ) ;
//    I2C_CHIPID = ( QMC5883_ADDR << 1 ) ;

// //     writeRegister(0x02,0x00); //select mode register - continuous measurement mode
     AT24CXX_WriteOneByte(0x02,0X00); //
#endif

         //

 // #define SNOOP_E  1
#ifdef  SNOOP_E
printf(": I2C Snoop : \r\n");

// #define    HMC5883_ADDR     (0x1E)    // HMC5883L

// // int  CHIP_ADDR ;

    for(int i=0x01; i<=0xFF; i+=2)
    {
// //      CHIP_ADDR = ( i ) ;

// //      printf( "* dvc%02X: r%02X = %02X  ,"   , i , 0    , readRegister(0)    );
// //      printf( "  dvc%02X: r%02X = %02X \r\n" , i , 0x75 , readRegister(0x75) );
      printf( "* dvc%02X: r%02X = %02X  ,"   , i , 0    , AT24CXX_ReadOneByte(0)    );
      printf( "  dvc%02X: r%02X = %02X \r\n" , i , 0x75 , AT24CXX_ReadOneByte(0x75) );

// //      if( readRegister(0) != 0xFF )
      if( AT24CXX_ReadOneByte( 0 ) != 0xFF )
      {
        for(int i=0x01; i<=0xFF; i+=2)
        {
// //          CHIP_ADDR = ( HMC5883_ADDR ) ;
//          printf( " %02X ," , readRegister(i)    );
          printf( " %02X ," , AT24CXX_ReadOneByte( i )    );
        }
        printf("\r\n");
      }
    }

#endif

         //

     //

     Delay_Ms(500);

    I2C_CHIPID = ( MPU_ADDR ) ;
    MPU6050_Write();

     Delay_Ms(500);

     printf("> mpu6050 Test ... \r\n");

    while( 1 )
    {

        I2C_CHIPID = ( MPU_ADDR ) ;
        MPU6050_TEST();

        I2C_CHIPID = ( HMC5883_ADDR ) ;
        HMC_Get_Compass();

        I2C_CHIPID = ( QMC5883_ADDR ) ;
        QMC_Get_Compass();

         Delay_Ms(500);

    }

}

///
