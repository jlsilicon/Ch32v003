/*
  Create project as : ch32v003 - f4p6  (- Not 103/203)
  - copy main.c from /EVT/EXAM/...

 WORKS :
*/


#include "debug.h"


/* Global define */


/* Global Variable */
vu8  val;


//


void ADCConfig(void) ;



/*
To configure the GPIO for ADC input at D4, which has an A7 channel connected, you need to code as shown below:

Macro can be used for ADC Pin and Port definition
*/

#define ANALOG1_PIN     GPIO_Pin_4

#define ANALOG1_PORT    GPIOD


uint16_t  adcReading ;
int       adcFlag = 1 ;


/*
int  main()
// void  setup()
{
    adcFlag = 0 ;

    ADCConfig();

  // }

// void  loop()
// {

  while(1)
  {

    if(adcFlag == 1)
    {
          adcReading = ADC_GetInjectedConversionValue( ADC1 , ADC_InjectedChannel_1 );
          adcReading = (adcReading * 325)/100; // Converting to real voltage w.r.t. VCC 3.3V, 0.985 multiplier factor is used with 3.3V for calibration. 330 * 0.985 = 325. This was as per my board, you might need to do it with different multiplier value.
          adcFlag = 0;

          printf(" AdcIn = %d \n" , adcReading );

          Delay_Ms(500);

    }

  }

return(0);
}
*/


void ADCConfig(void)
{
    ADC_InitTypeDef   ADC_InitStructure  = {0};
    GPIO_InitTypeDef  GPIO_InitStructure = {0};
    NVIC_InitTypeDef  NVIC_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    RCC_ADCCLKConfig(RCC_PCLK2_Div8);

    GPIO_InitStructure.GPIO_Pin  = ANALOG1_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(ANALOG1_PORT, &GPIO_InitStructure);

/*
Now, to configure the ADC to read the Pin D4/A7 in a discontinuous sensing mode with no external trigger
 (it��s a software trigger mode) and just a single channel to read,
 the following code needs to be written.
*/

    ADC_DeInit(ADC1);
    ADC_InitStructure.ADC_Mode               = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode       = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConv   = ADC_ExternalTrigInjecConv_None;
    ADC_InitStructure.ADC_DataAlign          = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel       = 1;
    ADC_Init(ADC1, &ADC_InitStructure);

/*
The following code is to configure the sequence length. As we have only one channel, we will use 1.
Then, channel 7 is linked to ADC data register 1.

In this example, we have used ADC_SampleTime_241Cycles for ADC pin sampling,
 more sampling time will give a better averaged reading as you are giving more time to connect and hold the circuit of ADC to charge the capacitor.
*/

    ADC_InjectedSequencerLengthConfig(ADC1, 1);
    ADC_InjectedChannelConfig(ADC1, ADC_Channel_7, 1, ADC_SampleTime_241Cycles);
    ADC_ExternalTrigInjectedConvCmd(ADC1, DISABLE);

/*
Then, we need to enable the interrupt so that after starting the ADC measurement,
it will trigger as soon as the conversion is completed
*/

    NVIC_InitStructure.NVIC_IRQChannel                   = ADC_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

/*
Finally, basic ADC calibration and Enable the ADC.
*/

    ADC_Calibration_Vol(ADC1, ADC_CALVOL_50PERCENT);
    ADC_ITConfig(ADC1, ADC_IT_JEOC, ENABLE);
    ADC_Cmd(ADC1, ENABLE);

    ADC_ResetCalibration(ADC1);

    while (ADC_GetResetCalibrationStatus(ADC1))
     {}

    ADC_StartCalibration(ADC1);
    while (ADC_GetCalibrationStatus(ADC1))
     {}

}

/*
To initiate the ADC conversion, you need to call the ADC_SoftwareStartInjectedConvCmd() function:

ADC_SoftwareStartInjectedConvCmd(ADC1, ENABLE);
To capture the interrupt once the conversion is complete, the following code is required for the IRQ handling.

I have made a Flag variable high and nothing else to avoid being in the interrupt function for too long.

The rest of the processing I did in the main code.
*/

void ADC1_IRQHandler() __attribute__((interrupt("WCH-Interrupt-fast")));

/**

 * The function ADC1_IRQHandler handles the interrupt for ADC1 and prints the value of the injected

 * conversion.

 */

void ADC1_IRQHandler()
{

    if (ADC_GetITStatus(ADC1, ADC_IT_JEOC) == SET)
    {
        adcFlag = 1;

        ADC_ClearITPendingBit(ADC1, ADC_IT_JEOC);

    }

}

/*
The main code will look like this,
 where we will read ADC reading and further process it to convert it to real-world voltage level.

325 constant used below is 3.25V observed on board board for VCC.
If it is different for your board, you need to use that.
It is advisable to use a precision(1% or better) LDO or VREF IC for powering the MCU if you need best possible accuracy.

if(adcFlag ==1)
        {
          adcReading = ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_1);
          adcReading = (adcReading * 325)/100; // Converting to real voltage w.r.t. VCC 3.3V, 0.985 multiplier factor is used with 3.3V for calibration. 330 * 0.985 = 325. This was as per my board, you might need to do it with different multiplier value.
          adcFlag = 0;
        }
*/


//


/*********************************************************************
 * @fn      USARTx_CFG
 *
 * @brief   Initializes the USART2 & USART3 peripheral.
 *
 * @return  none
 */
void USARTx_CFG(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure = {0};
    USART_InitTypeDef USART_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_USART1, ENABLE);

    /* USART1 TX-->D.5   RX-->D.6 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;

    USART_Init(USART1, &USART_InitStructure);
    USART_Cmd(USART1, ENABLE);
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
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    SystemCoreClockUpdate();
    Delay_Init();

//    USARTx_CFG();

    USART_Printf_Init(115200);
    printf("SystemClk:%d\r\n",SystemCoreClock);
    printf( "ChipID:%08x\r\n", DBGMCU_GetCHIPID() );

    adcFlag = 1 ;

    ADCConfig();

    ;

    while(1)
    {

        if(adcFlag == 1)
        {
              adcReading = ADC_GetInjectedConversionValue( ADC1 , ADC_InjectedChannel_1 );
              adcReading = (adcReading * 325)/100; // Converting to real voltage w.r.t. VCC 3.3V, 0.985 multiplier factor is used with 3.3V for calibration. 330 * 0.985 = 325. This was as per my board, you might need to do it with different multiplier value.
              adcFlag = 0;

               printf(" AdcIn = %d \r\n" , adcReading );

              ADC_SoftwareStartInjectedConvCmd(ADC1, ENABLE);

               Delay_Ms(500);
        }

    }

    //

/*
            while(USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET)
            {
                // waiting for receiving finish //
            }
            val = (USART_ReceiveData(USART1));
            USART_SendData(USART1, ~val);
            while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
            {
                // waiting for sending finish //
            }
*/

    //

return(0);
}

///
