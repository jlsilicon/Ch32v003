/*

The VCC and GND pins of the CH32V103 development board are connected to the VCC and GND pins of the MPU6050 module.
 PA1 = SDA : The PA1 pin of the CH32V103 development board is connected to the SDA pin of the MPU6050 module
 PA2 = SCL : The PA2 pin of the CH32V103 development board is connected to the SCL pin of the MPU6050 module

Mpu6050 : Id Address = 0x68 wrt , 0x69 rd

 - Compiles Downloads , Returns same data : Temp=175, AX=-11823 , AY=-11823 , AZ=-11823 , GX=-11823 , AGY=-11823 , GZ=-11823

 - TX = PA9
 - RX = PA10

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
#define MPU_AD0_H               GPIO_SetBits(  GPIOA,GPIO_Pin_7)    // SDA
#define MPU_AD0_L               GPIO_ResetBits(GPIOA,GPIO_Pin_7)  // SDA
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

// MPU6050.c //

// #include "mpu6050.h"
/*
#include "debug.h"
*/

//

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

return MPU_Set_LPF( rate / 2 ); //LPF
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


// int main(void)
// int main_1(void)
int  MPU6050_TEST(void)
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

        while( 1 )
        {
            temp = MPU_Get_Temperature(); //
            MPU_Get_Accelerometer(&aacx,&aacy,&aacz);   //
            MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);    //

            printf("\r\n Temperature:%d \r\n",temp);

             Delay_Ms(500);

        }
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
 *
 * @brief   Initializes the IIC peripheral.
 *
 * @return  none
 */
void IIC_Init(u32 bound, u16 address)
{
printf("> IIC_Init() :\r\n");

    GPIO_InitTypeDef GPIO_InitStructure={0};
    I2C_InitTypeDef I2C_InitTSturcture={0};

    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE );
    RCC_APB1PeriphClockCmd( RCC_APB1Periph_I2C1, ENABLE );

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOC, &GPIO_InitStructure );

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOC, &GPIO_InitStructure );

    I2C_InitTSturcture.I2C_ClockSpeed = bound;
    I2C_InitTSturcture.I2C_Mode = I2C_Mode_I2C;
    I2C_InitTSturcture.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitTSturcture.I2C_OwnAddress1 = address;
    I2C_InitTSturcture.I2C_Ack = I2C_Ack_Enable;
    I2C_InitTSturcture.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init( I2C1, &I2C_InitTSturcture );

    I2C_Cmd( I2C1, ENABLE );

    I2C_AcknowledgeConfig( I2C1, ENABLE );

printf("< IIC_Init() .\r\n");
}

/*********************************************************************
 * @fn      AT24CXX_Init
 *
 * @brief   Initializes AT24xx EEPROM.
 *
 * @return  none
 */
void AT24CXX_Init(void)
{
    IIC_Init( 100000, 0xA0 );
}

void MPU6050_Init(void)
{
    IIC_Init( 100000, 0x68 );
}


/*********************************************************************
 * @fn      AT24CXX_ReadOneByte
 *
 * @brief   Read one data from EEPROM.
 *
 * @param   ReadAddr - Read first address.
 *
 * @return  temp - Read data.
 */
u8 AT24CXX_ReadOneByte(u16 ReadAddr)
{
    u8 temp=0;

    while( I2C_GetFlagStatus( I2C1, I2C_FLAG_BUSY ) != RESET );
    I2C_GenerateSTART( I2C1, ENABLE );

    while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_MODE_SELECT ) );
// //    I2C_Send7bitAddress( I2C1, 0XA0, I2C_Direction_Transmitter );
    I2C_Send7bitAddress( I2C1, 0X68, I2C_Direction_Transmitter );

    while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED ) );

/*
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
*/

    I2C_GenerateSTART( I2C1, ENABLE );

    while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_MODE_SELECT ) );
// //    I2C_Send7bitAddress( I2C1, 0XA0, I2C_Direction_Receiver );
    I2C_Send7bitAddress( I2C1, 0X68, I2C_Direction_Receiver );

    while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED ) );
  while( I2C_GetFlagStatus( I2C1, I2C_FLAG_RXNE ) ==  RESET )
    I2C_AcknowledgeConfig( I2C1, DISABLE );

    temp = I2C_ReceiveData( I2C1 );
  I2C_GenerateSTOP( I2C1, ENABLE );

    return temp;
}


/*********************************************************************
 * @fn      AT24CXX_WriteOneByte
 *
 * @brief   Write one data to EEPROM.
 *
 * @param   WriteAddr - Write frist address.
 *
 * @return  DataToWrite - Write data.
 */
void AT24CXX_WriteOneByte(u16 WriteAddr, u8 DataToWrite)
{
    while( I2C_GetFlagStatus( I2C1, I2C_FLAG_BUSY ) != RESET );
    I2C_GenerateSTART( I2C1, ENABLE );

    while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_MODE_SELECT ) );
// //    I2C_Send7bitAddress( I2C1, 0XA0, I2C_Direction_Transmitter );
    I2C_Send7bitAddress( I2C1, 0X68, I2C_Direction_Transmitter );

    while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED ) );

/*
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
*/

    if( I2C_GetFlagStatus( I2C1, I2C_FLAG_TXE ) !=  RESET )
    {
        I2C_SendData( I2C1, DataToWrite );
    }

    while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED ) );
    I2C_GenerateSTOP( I2C1, ENABLE );
}


/*********************************************************************
 * @fn      AT24CXX_Read
 *
 * @brief   Read multiple data from EEPROM.
 *
 * @param   ReadAddr - Read frist address. (AT24c02: 0~255)
 *          pBuffer - Read data.
 *          NumToRead - Data number.
 *
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
 *
 * @brief   Write multiple data to EEPROM.
 *
 * @param   WriteAddr - Write frist address. (AT24c02: 0~255)
 *          pBuffer - Write data.
 *          NumToWrite - Data number.
 *
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
 *
 * @brief   Main program.
 *
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

// //    AT24CXX_Init();
    MPU6050_Init();

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

    MPU6050_Write();

    while( 1 )
    {

        printf("> mpu6050 Test ... \r\n");

        MPU6050_TEST();

         Delay_Ms(500);

    }

}

///
