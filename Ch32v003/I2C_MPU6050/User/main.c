/*

The VCC and GND pins of the CH32V103 development board are connected to the VCC and GND pins of the MPU6050 module.
 PA1 = SDA : The PA1 pin of the CH32V103 development board is connected to the SDA pin of the MPU6050 module
 PA2 = SCL : The PA2 pin of the CH32V103 development board is connected to the SCL pin of the MPU6050 module

 - Compiles Downloads , Returns same data : Temp=175, AX=-11823 , AY=-11823 , AZ=-11823 , GX=-11823 , AGY=-11823 , GZ=-11823

 - TX = PA9
 - RX = PA10

*/


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


//#define MPU_ACCEL_OFFS_REG        0X06    //accel_offs寄存器,可读取版本号,寄存器手册未提到
//#define MPU_PROD_ID_REG           0X0C    //prod id寄存器,在寄存器手册未提到
#define MPU_SELF_TESTX_REG      0X0D    //自检寄存器X
#define MPU_SELF_TESTY_REG      0X0E    //自检寄存器Y
#define MPU_SELF_TESTZ_REG      0X0F    //自检寄存器Z
#define MPU_SELF_TESTA_REG      0X10    //自检寄存器A
#define MPU_SAMPLE_RATE_REG     0X19    //采样频率分频器
#define MPU_CFG_REG             0X1A    //配置寄存器
#define MPU_GYRO_CFG_REG        0X1B    //陀螺仪配置寄存器
#define MPU_ACCEL_CFG_REG       0X1C    //加速度计配置寄存器
#define MPU_MOTION_DET_REG      0X1F    //运动检测阀值设置寄存器
#define MPU_FIFO_EN_REG         0X23    //FIFO使能寄存器
#define MPU_I2CMST_CTRL_REG     0X24    //IIC主机控制寄存器
#define MPU_I2CSLV0_ADDR_REG    0X25    //IIC从机0器件地址寄存器
#define MPU_I2CSLV0_REG         0X26    //IIC从机0数据地址寄存器
#define MPU_I2CSLV0_CTRL_REG    0X27    //IIC从机0控制寄存器
#define MPU_I2CSLV1_ADDR_REG    0X28    //IIC从机1器件地址寄存器
#define MPU_I2CSLV1_REG         0X29    //IIC从机1数据地址寄存器
#define MPU_I2CSLV1_CTRL_REG    0X2A    //IIC从机1控制寄存器
#define MPU_I2CSLV2_ADDR_REG    0X2B    //IIC从机2器件地址寄存器
#define MPU_I2CSLV2_REG         0X2C    //IIC从机2数据地址寄存器
#define MPU_I2CSLV2_CTRL_REG    0X2D    //IIC从机2控制寄存器
#define MPU_I2CSLV3_ADDR_REG    0X2E    //IIC从机3器件地址寄存器
#define MPU_I2CSLV3_REG         0X2F    //IIC从机3数据地址寄存器
#define MPU_I2CSLV3_CTRL_REG    0X30    //IIC从机3控制寄存器
#define MPU_I2CSLV4_ADDR_REG    0X31    //IIC从机4器件地址寄存器
#define MPU_I2CSLV4_REG         0X32    //IIC从机4数据地址寄存器
#define MPU_I2CSLV4_DO_REG      0X33    //IIC从机4写数据寄存器
#define MPU_I2CSLV4_CTRL_REG    0X34    //IIC从机4控制寄存器
#define MPU_I2CSLV4_DI_REG      0X35    //IIC从机4读数据寄存器

#define MPU_I2CMST_STA_REG      0X36    //IIC主机状态寄存器
#define MPU_INTBP_CFG_REG       0X37    //中断/旁路设置寄存器
#define MPU_INT_EN_REG          0X38    //中断使能寄存器
#define MPU_INT_STA_REG         0X3A    //中断状态寄存器

#define MPU_ACCEL_XOUTH_REG     0X3B    //加速度值,X轴高8位寄存器
#define MPU_ACCEL_XOUTL_REG     0X3C    //加速度值,X轴低8位寄存器
#define MPU_ACCEL_YOUTH_REG     0X3D    //加速度值,Y轴高8位寄存器
#define MPU_ACCEL_YOUTL_REG     0X3E    //加速度值,Y轴低8位寄存器
#define MPU_ACCEL_ZOUTH_REG     0X3F    //加速度值,Z轴高8位寄存器
#define MPU_ACCEL_ZOUTL_REG     0X40    //加速度值,Z轴低8位寄存器

#define MPU_TEMP_OUTH_REG       0X41    //温度值高八位寄存器
#define MPU_TEMP_OUTL_REG       0X42    //温度值低8位寄存器

#define MPU_GYRO_XOUTH_REG      0X43    //陀螺仪值,X轴高8位寄存器
#define MPU_GYRO_XOUTL_REG      0X44    //陀螺仪值,X轴低8位寄存器
#define MPU_GYRO_YOUTH_REG      0X45    //陀螺仪值,Y轴高8位寄存器
#define MPU_GYRO_YOUTL_REG      0X46    //陀螺仪值,Y轴低8位寄存器
#define MPU_GYRO_ZOUTH_REG      0X47    //陀螺仪值,Z轴高8位寄存器
#define MPU_GYRO_ZOUTL_REG      0X48    //陀螺仪值,Z轴低8位寄存器

#define MPU_I2CSLV0_DO_REG      0X63    //IIC从机0数据寄存器
#define MPU_I2CSLV1_DO_REG      0X64    //IIC从机1数据寄存器
#define MPU_I2CSLV2_DO_REG      0X65    //IIC从机2数据寄存器
#define MPU_I2CSLV3_DO_REG      0X66    //IIC从机3数据寄存器

#define MPU_I2CMST_DELAY_REG    0X67    //IIC主机延时管理寄存器
#define MPU_SIGPATH_RST_REG     0X68    //信号通道复位寄存器
#define MPU_MDETECT_CTRL_REG    0X69    //运动检测控制寄存器
#define MPU_USER_CTRL_REG       0X6A    //用户控制寄存器
#define MPU_PWR_MGMT1_REG       0X6B    //电源管理寄存器1
#define MPU_PWR_MGMT2_REG       0X6C    //电源管理寄存器2
#define MPU_FIFO_CNTH_REG       0X72    //FIFO计数寄存器高八位
#define MPU_FIFO_CNTL_REG       0X73    //FIFO计数寄存器低八位
#define MPU_FIFO_RW_REG         0X74    //FIFO读写寄存器
#define MPU_DEVICE_ID_REG       0X75    //器件ID寄存器

#define MPU_ADDR                0X68

//#define MPU_READ    0XD1

//#define MPU_WRITE   0XD0

u8 MPU_Init(void);                              //初始化MPU6050
u8 MPU_Write_Len(u8 addr,u8 reg,u8 len,u8 *buf);//IIC连续写
u8 MPU_Read_Len(u8 addr,u8 reg,u8 len,u8 *buf); //IIC连续读
u8 MPU_Write_Byte(u8 reg,u8 data);              //IIC写一个字节
u8 MPU_Read_Byte(u8 reg);                       //IIC读一个字节

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


// #include "mpu6050.h"
#include "debug.h"

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

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);     //AFIO时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);    //IO PORTA时钟

//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;              // 端口配置
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;              // 端口配置
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;        //推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;       //IO口速度为50MHz
    GPIO_Init(GPIOA, &GPIO_InitStructure);                  //GPIOA

    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE); //禁止JTAG,从而PA15可以做普通IO使用,否则PA15不能做普通O!!!

    MPU_AD0_L;                              //控制MPU6050的AD0脚为低电平,从机地址为:0X68

    IIC_Init();                             //初始化IIC总线

    MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X80); //复位MPU6050
    Delay_Ms(100);
    MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X00); //唤醒MPU6050
    MPU_Set_Gyro_Fsr(3);                    //陀螺仪传感器,±2000dps
    MPU_Set_Accel_Fsr(0);                   //加速度传感器,±2g
    MPU_Set_Rate(50);                       //设置采样率50Hz
    MPU_Write_Byte(MPU_INT_EN_REG,0X00);    //关闭所有中断
    MPU_Write_Byte(MPU_USER_CTRL_REG,0X00); //I2C主模式关闭
    MPU_Write_Byte(MPU_FIFO_EN_REG,0X00);   //关闭FIFO
    MPU_Write_Byte(MPU_INTBP_CFG_REG,0X80); //INT引脚低电平有效

    res=MPU_Read_Byte(MPU_DEVICE_ID_REG);
    if(res==MPU_ADDR)//器件ID正确
    {
        MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X01); //设置CLKSEL,PLL X轴为参考
        MPU_Write_Byte(MPU_PWR_MGMT2_REG,0X00); //加速度与陀螺仪都工作
        MPU_Set_Rate(50);                       //设置采样率为50Hz
    }
    else
      return 1;
    return 0;
}

*/

void  IIC_Init(u32 bound, u16 address)
// u8  MPU_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure={0};
    I2C_InitTypeDef I2C_InitTSturcture={0};

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

    I2C_InitTSturcture.I2C_ClockSpeed  = bound;
    I2C_InitTSturcture.I2C_Mode        = I2C_Mode_I2C;
    I2C_InitTSturcture.I2C_DutyCycle   = I2C_DutyCycle_2;
    I2C_InitTSturcture.I2C_OwnAddress1 = address;
    I2C_InitTSturcture.I2C_Ack         = I2C_Ack_Enable;
    I2C_InitTSturcture.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;

    I2C_Init( I2C1, &I2C_InitTSturcture );

    I2C_Cmd( I2C1, ENABLE );
//    IIC_SendByte( ENABLE ) ;  // - ???

//    I2C_AcknowledgeConfig( I2C1, ENABLE );
//    IIC_WaitAck();     //等待应答 // - ???
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
    if(res == MPU_ADDR)//器件ID正确
    {
        MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X01); //设置CLKSEL,PLL X轴为参考
        MPU_Write_Byte(MPU_PWR_MGMT2_REG,0X00); //加速度与陀螺仪都工作
        MPU_Set_Rate(50);                       //设置采样率为50Hz
    }
//    else
//      return 1;
//    return 0;

}


//fsr:0,±250dps;1,±500dps;2,±1000dps;3,±2000dps
u8 MPU_Set_Gyro_Fsr(u8 fsr)
{
    return MPU_Write_Byte(MPU_GYRO_CFG_REG,fsr<<3);//设置陀螺仪满量程范围
}

u8 MPU_Set_Accel_Fsr(u8 fsr)
{
    return MPU_Write_Byte(MPU_ACCEL_CFG_REG,fsr<<3);//设置加速度传感器满量程范围
}

u8 MPU_Set_LPF(u16 lpf)
{
    u8 data=0;
    if(lpf>=188)data=1;
    else if(lpf>=98)data=2;
    else if(lpf>=42)data=3;
    else if(lpf>=20)data=4;
    else if(lpf>=10)data=5;
    else data=6;
    return MPU_Write_Byte(MPU_CFG_REG,data);//设置数字低通滤波器
}


u8 MPU_Set_Rate(u16 rate)
{
u8 data;

if(rate>1000)rate=1000;
    if(rate<4)rate=4;
    data=1000/rate-1;
    data=MPU_Write_Byte(MPU_SAMPLE_RATE_REG,data);  //设置数字低通滤波器

    return MPU_Set_LPF(rate/2); //自动设置LPF为采样率的一半
}


short MPU_Get_Temperature(void)
{
    u8 buf[2];
    short raw;
    float temp;
    MPU_Read_Len(MPU_ADDR,MPU_TEMP_OUTH_REG,2,buf);
    raw=((u16)buf[0]<<8)|buf[1];
    temp=36.53+((double)raw)/340;
    return temp*100;;
}


u8 MPU_Get_Gyroscope(short *gx,short *gy,short *gz)
{
    u8 buf[6],res;
    res=MPU_Read_Len(MPU_ADDR,MPU_GYRO_XOUTH_REG,6,buf);
    if(res==0)
    {
        *gx=((u16)buf[0]<<8)|buf[1];
        *gy=((u16)buf[2]<<8)|buf[3];
        *gz=((u16)buf[4]<<8)|buf[5];
        printf("GYRO:  X=%d   Y=%d   Z=%d  \n",*gx,*gy,*gz);
    }
    return res;;
}


u8 MPU_Get_Accelerometer(short *ax,short *ay,short *az)
{
    u8 buf[6],res;
    res=MPU_Read_Len(MPU_ADDR,MPU_ACCEL_XOUTH_REG,6,buf);
    if(res==0)
    {
        *ax=((u16)buf[0]<<8)|buf[1];
        *ay=((u16)buf[2]<<8)|buf[3];
        *az=((u16)buf[4]<<8)|buf[5];
        printf("ACC:  X=%d   Y=%d   Z=%d  \n",*ax,*ay,*az);
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

    IIC_SendByte(reg); //写寄存器地址
    IIC_WaitAck();     //等待应答

    for(i=0;i<len;i++)
    {
        IIC_SendByte(buf[i]);  //发送数据
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
    IIC_SendByte((addr<<1)|0);//发送器件地址+写命令
    if(IIC_WaitAck())  //等待应答
    {
        IIC_Stop();
        return 1;
    }
    IIC_SendByte(reg); //写寄存器地址
    IIC_WaitAck();     //等待应答

    IIC_Start();
    IIC_SendByte((addr<<1)|1);//发送器件地址+读命令
    IIC_WaitAck();     //等待应答
    while(len)
    {
        if(len==1)
            * buf = IIC_ReadByte(0);//读数据,发送nACK
        else
            * buf = IIC_ReadByte(1);     //读数据,发送ACK
        len--;
        buf++;
    }
    IIC_Stop(); // STOP
    return 0;
}


u8 MPU_Write_Byte(u8 reg,u8 data)
{
    IIC_Start();
    IIC_SendByte((MPU_ADDR<<1)|0);//发送器件地址+写命令
    if(IIC_WaitAck())  //等待应答
    {
        IIC_Stop();
        return 1;
    }
    IIC_SendByte(reg); //写寄存器地址
    IIC_WaitAck();     //等待应答
    IIC_SendByte(data);//发送数据
    if(IIC_WaitAck())  //等待ACK
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
     IIC_SendByte((MPU_ADDR<<1)|0);//发送器件地址+写命令
      IIC_WaitAck();     //等待应答
     IIC_SendByte(reg); //写寄存器地址
      IIC_WaitAck();     //等待应答

    IIC_Start();
     IIC_SendByte((MPU_ADDR<<1)|1);//发送器件地址+读命令
      IIC_WaitAck();     //等待应答
     res=IIC_ReadByte(0);//读取数据,发送nACK

    IIC_Stop();         // STOP

return res;
}


int main(void)
{
    short aacx,aacy,aacz;       //加速度传感器原始数据
    short gyrox,gyroy,gyroz;    //陀螺仪原始数据
    short temp;                 //温度

        NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
        Delay_Init();
        USART_Printf_Init(115200);
//        MPU_Init();                //MPU6050
        IIC_Init( 100000, 0xA0);

        printf("SystemClk:%d\r\n",SystemCoreClock);
        printf("MPU6050 TEST\r\n");

        while(1)
        {
            temp = MPU_Get_Temperature(); //得到温度值
            MPU_Get_Accelerometer(&aacx,&aacy,&aacz);   //得到加速度传感器数据
            MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);    //得到陀螺仪数据

            printf("Temperature:%d\r\n",temp);

             Delay_Ms(500);

        }
}

///
