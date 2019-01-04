/**
  ******************************************************************************
  * @file    ADC_DMA/main.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    23-March-2012
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx.h"
#include "stm32f0_discovery.h"
#include <stdio.h>
#include <stdlib.h>
#define MAVLINK        250
/** @addtogroup STM32F0_Discovery_Peripheral_Examples
  * @{
  */

/** @addtogroup ADC_DMA
  * @{
  */
	/** @addtogroup HyperTerminal_Interrupt
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern uint8_t TxBuffer[];
extern uint8_t RxBuffer[];
extern __IO uint8_t TxCount;
extern __IO uint8_t TxLength;
extern __IO uint8_t RxHead;
extern __IO uint8_t RxEnd;
extern uint8_t TxBuffer2[];
extern uint8_t RxBuffer2[];
extern __IO uint8_t TxCount2;
extern __IO uint8_t TxLength2;
extern __IO uint8_t RxHead2;
extern __IO uint8_t RxEnd2;
extern __IO uint16_t RegularConvData_Tab[];
extern union{
uint16_t RemoteData_Tab[10];
uint8_t  SendData_tab[20];
}ap;
extern ITStatus Time2_5ms;
extern ITStatus Time10ms;
uint16_t count = 0;
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define ADC1_DR_Address                0x40012440

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
__IO uint32_t TempSensVoltmv = 0, VrefIntVoltmv = 0;
__IO uint16_t throttle = 0,yaw = 0, roll = 0, pitch = 0;
uint8_t ADCount = 0;

/* Private function prototypes -----------------------------------------------*/
static void ADC1_CH_DMA_Config(void);
static void TIM_Config(void);
static void NVIC_Config(void);
static void USART_Config(void);
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
  /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f0xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f0xx.c file
     */
  
  /* ADC1 channel with DMA configuration */
	uint16_t i,k,tmpdata;
	uint8_t sum;
  ADC1_CH_DMA_Config();
	TIM_Config();
	NVIC_Config();
	USART_Config();
	//USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
	//USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	STM_EVAL_LEDInit(LED3);
	STM_EVAL_PBInit(BUTTON_USER,BUTTON_MODE_GPIO);
  STM_EVAL_LEDOn(LED3);
  /* Infinite loop */
  while (1)
  {
    /* Test DMA1 TC flag */
		srand(10);
		k = rand();
    if((DMA_GetFlagStatus(DMA1_FLAG_TC1)) != RESET ){
			/* Clear DMA TC flag */
			DMA_ClearFlag(DMA1_FLAG_TC1);
		}			
    if(Time2_5ms == SET){
			for(i = 0; i < 4; i++){
				ap.RemoteData_Tab[i] += (uint16_t)((uint32_t)((RegularConvData_Tab[i]* 1000) / 0xFFF)+1000);
			}
			ADCount++;
			Time2_5ms = RESET;
		}
		if(Time10ms == SET){
			if(TxCount == TxLength){
				for(i = 0; i < 4; i++){
					ap.RemoteData_Tab[i] /= ADCount;
				}
				ADCount = 1;
				i = 0;
				sum = 0;
				TxBuffer[i++] = 0x2a;
				TxBuffer[i++] = 0x0;
				for(k = 0; k < 20; k++){
					TxBuffer[i] = ap.SendData_tab[k];
					sum += TxBuffer[i++];
				}
				//i = 0;
				if(RxEnd2 != RxHead2){
					tmpdata = RxHead2;
					if(tmpdata < RxEnd2){
						for(k = RxEnd2; k < MAVLINK; k++){
							TxBuffer[i++] = RxBuffer2[k];
							sum += RxBuffer2[k];
						}
						for(k = 0; k < tmpdata; k++){
							TxBuffer[i++] = RxBuffer2[k];
							sum += RxBuffer2[k];
						}
					}else{
						for(k = RxEnd2; k < tmpdata; k++){
							TxBuffer[i++] = RxBuffer2[k];
							sum += RxBuffer2[k];
						}
					}					
					RxEnd2 = tmpdata;
				}
				TxBuffer[0] =i + 2;
				sum += TxBuffer[0];
				TxBuffer[i++] = sum;
				TxBuffer[i++] = 0x23;
				TxCount = 0;
				TxLength = i;	
				USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
			}
			if((RxEnd != RxHead) && (TxCount2 == TxLength2)){
				tmpdata = RxHead;
				i = 0;
				if(tmpdata < RxEnd){
					for(k = RxEnd; k < MAVLINK; k++){
						TxBuffer2[i++] = RxBuffer[k];
					}
					for(k = 0; k < tmpdata; k++){
						TxBuffer2[i++] = RxBuffer[k];
					}
				}else{
					for(k = RxEnd; k < tmpdata; k++){
						TxBuffer2[i++] = RxBuffer[k];
					}
				}
				RxEnd = tmpdata;
				TxCount2 = 0;
				TxLength2 = i;	
				USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
			}
			Time10ms = RESET;
		}
    
    //button = STM_EVAL_PBGetState(BUTTON_USER);
		/*if(button != 0){
			STM_EVAL_LEDOff(LED3);
		}else{
			STM_EVAL_LEDOn(LED3);
		}*/
//		pitch = (uint16_t)((uint32_t)((RegularConvData_Tab[0]* 1000) / 0xFFF)+1000);
//		roll = (uint16_t)((uint32_t)((RegularConvData_Tab[1]* 1000) / 0xFFF)+1000);
//		throttle = (uint16_t)((uint32_t)((RegularConvData_Tab[2]* 1000) / 0xFFF)+1000);
//		yaw = (uint16_t)((uint32_t)((RegularConvData_Tab[3]* 1000) / 0xFFF)+1000);
    /* Convert temperature sensor voltage value in mv */
    //TempSensVoltmv = (uint32_t)((RegularConvData_Tab[4]* 3300) / 0xFFF);
    
    /* Convert Vref voltage value in mv */
    //VrefIntVoltmv  = (uint32_t)((RegularConvData_Tab[5]* 3300) / 0xFFF);  
  }
}

/**
  * @brief  ADC1 channel with DMA configuration
  * @param  None
  * @retval None
  */
static void ADC1_CH_DMA_Config(void)
{
  ADC_InitTypeDef     ADC_InitStructure;
  DMA_InitTypeDef     DMA_InitStructure;
	GPIO_InitTypeDef    GPIO_InitStructure;
  uint8_t i;
  /* ADC1 DeInit */  
  ADC_DeInit(ADC1);
  
  /* ADC1 Periph clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
  
  /* DMA1 clock enable */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1 , ENABLE);
  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
	
  /* DMA1 Channel1 Config */
  DMA_DeInit(DMA1_Channel1);
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC1_DR_Address;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)RegularConvData_Tab;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = 4;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);
  

  
  /* Initialize ADC structure */
  ADC_StructInit(&ADC_InitStructure);
  
  /* Configure the ADC1 in continous mode withe a resolutuion equal to 12 bits  */
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE; 
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_ScanDirection = ADC_ScanDirection_Upward;
  ADC_Init(ADC1, &ADC_InitStructure); 
 
  ADC_ChannelConfig(ADC1, ADC_Channel_10 , ADC_SampleTime_239_5Cycles);
	ADC_ChannelConfig(ADC1, ADC_Channel_11 , ADC_SampleTime_239_5Cycles);
	ADC_ChannelConfig(ADC1, ADC_Channel_12 , ADC_SampleTime_239_5Cycles);
	ADC_ChannelConfig(ADC1, ADC_Channel_13 , ADC_SampleTime_239_5Cycles);
  /* Convert the ADC1 temperature sensor  with 55.5 Cycles as sampling time */ 
  ADC_ChannelConfig(ADC1, ADC_Channel_TempSensor , ADC_SampleTime_55_5Cycles);  
  ADC_TempSensorCmd(ENABLE);
  
  /* Convert the ADC1 Vref  with 55.5 Cycles as sampling time */ 
  ADC_ChannelConfig(ADC1, ADC_Channel_Vrefint , ADC_SampleTime_55_5Cycles); 
  ADC_VrefintCmd(ENABLE);
  
  /* ADC Calibration */
  ADC_GetCalibrationFactor(ADC1);
  
  /* Enable ADC1 */
  ADC_Cmd(ADC1, ENABLE);     
  
  /* Wait the ADCEN falg */
  while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_ADEN)); 
	
	  /* DMA1 Channel1 enable */
  DMA_Cmd(DMA1_Channel1, ENABLE);
  
  /* ADC DMA request in circular mode */
  ADC_DMARequestModeConfig(ADC1, ADC_DMAMode_Circular);
  
  /* Enable ADC_DMA */
  ADC_DMACmd(ADC1, ENABLE);  
  
  /* ADC1 regular Software Start Conv */ 
  ADC_StartOfConversion(ADC1);
	for(i = 0; i < 4; i++){
		ap.RemoteData_Tab[i] = 0;
	}
	for(; i < 10; i++){
		ap.RemoteData_Tab[i] = 1500;
	}
}

static void TIM_Config(void)
{
	__IO uint16_t CCR3_Val = 60000;
	__IO uint16_t CCR4_Val = 1500;
	uint16_t PrescalerValue = 0;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  /* TIM3 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

  /* Enable the TIM3 gloabal Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	PrescalerValue = (uint16_t) (SystemCoreClock  / 6000000) - 1;

  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 65535;
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

  /* Prescaler configuration */
  TIM_PrescalerConfig(TIM3, PrescalerValue, TIM_PSCReloadMode_Immediate);

  /* Output Compare Timing Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  /* Output Compare Timing Mode configuration: Channel3 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR3_Val;

  TIM_OC3Init(TIM3, &TIM_OCInitStructure);

  TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Disable);

  /* Output Compare Timing Mode configuration: Channel4 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR4_Val;

  TIM_OC4Init(TIM3, &TIM_OCInitStructure);

  TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Disable);
   
  /* TIM Interrupts enable */
  TIM_ITConfig(TIM3, TIM_IT_CC3 | TIM_IT_CC4, ENABLE);

  /* TIM3 enable counter */
  TIM_Cmd(TIM3, ENABLE);
}

/**
  * @brief  Configures the nested vectored interrupt controller.
  * @param  None
  * @retval None
  */
static void NVIC_Config(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Enable the USART Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

/**
  * @brief Configure the USART Device
  * @param  None
  * @retval None
  */
static void USART_Config(void)
{
  USART_InitTypeDef USART_InitStructure;
	GPIO_InitTypeDef    GPIO_InitStructure;
    
/* USARTx configured as follow:
  - BaudRate = 9600 baud  
  - Word Length = 8 Bits
  - Two Stop Bit
  - Odd parity
  - Hardware flow control disabled (RTS and CTS signals)
  - Receive and transmit enabled
  */
	USART_DeInit(USART1);
	USART_DeInit(USART2);
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	
	/* Connect PXx to USARTx_Tx */
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_1);
	/* Connect PXx to USARTx_Rx */
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_1);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_9 | GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_2;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART1, &USART_InitStructure);
	USART_Init(USART2, &USART_InitStructure);
	USART_OverrunDetectionConfig(USART1,USART_OVRDetection_Disable);
	USART_OverrunDetectionConfig(USART2,USART_OVRDetection_Disable);
	USART_Cmd(USART2, ENABLE);
	USART_Cmd(USART1, ENABLE);
}
#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
