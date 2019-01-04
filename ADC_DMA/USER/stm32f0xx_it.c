/**
  ******************************************************************************
  * @file    ADC_DMA/stm32f0xx_it.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    23-March-2012
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
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
#include "stm32f0xx_it.h"
#include "stm32f0_discovery.h"

/** @addtogroup STM32F0_Discovery_Peripheral_Examples
  * @{
  */
	/** @addtogroup HyperTerminal_Interrupt
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define TXBUFFERSIZE   (countof(TxBuffer) - 1)
#define RXBUFFERSIZE   0x20

/* Private macro -------------------------------------------------------------*/
#define countof(a)   (sizeof(a) / sizeof(*(a)))

/* Private variables ---------------------------------------------------------*/
uint8_t TxBuffer[MAVLINK];
uint8_t RxBuffer[MAVLINK];
__IO uint8_t TxCount = 0;
__IO uint8_t TxLength = 0;
__IO uint8_t RxHead = 0;
__IO uint8_t RxEnd = 0;
uint8_t TxBuffer2[MAVLINK];
uint8_t RxBuffer2[MAVLINK];
__IO uint8_t TxCount2 = 0;
__IO uint8_t TxLength2 = 0;
__IO uint8_t RxHead2 = 0;
__IO uint8_t RxEnd2 = 0;


/** @addtogroup ADC_DMA
  * @{
  */
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
__IO uint16_t CCR3_Val = 60000;
__IO uint16_t CCR4_Val = 15000;
uint16_t capture = 0;
uint16_t counter2 = 0;
__IO uint16_t RegularConvData_Tab[4];
union{
uint16_t RemoteData_Tab[10];
uint8_t  SendData_tab[20];
}ap;
ITStatus Time2_5ms = RESET;
ITStatus Time10ms  = RESET;
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M0 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
}

/******************************************************************************/
/*                 STM32F0xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f0xx.s).                                               */
/******************************************************************************/
/**
  * @brief  This function handles TIM3 global interrupt request.
  * @param  None
  * @retval None
  */
void TIM3_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM3, TIM_IT_CC3) != RESET)
  {
    TIM_ClearITPendingBit(TIM3, TIM_IT_CC3);
		Time10ms = SET;
    /* LED3 toggling with frequency = 219.7 Hz */
    STM_EVAL_LEDToggle(LED3);
    capture = TIM_GetCapture3(TIM3);
    TIM_SetCompare3(TIM3, capture + CCR3_Val);
  }
  else
  {
    TIM_ClearITPendingBit(TIM3, TIM_IT_CC4);
		
    Time2_5ms = SET;
    capture = TIM_GetCapture4(TIM3);
    TIM_SetCompare4(TIM3, capture + CCR4_Val);
  }
}
/**
  * @brief  This function handles USART1 global interrupt request.
  * @param  None
  * @retval None
  */
void USART1_IRQHandler(void)
{
  if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
  {
    /* Read one byte from the receive data register */
    RxBuffer[RxHead++] = (USART_ReceiveData(USART1) & 0x7F);
		if(RxHead >= MAVLINK)
    {
      /* Disable the EVAL_COM1 Receive interrupt */
      RxHead = 0;
    }
		if(RxHead == RxEnd){
			RxHead = 0;
			RxEnd = 0;
		}
  }

  if(USART_GetITStatus(USART1, USART_IT_TXE) != RESET)
  {   
    /* Write one byte to the transmit data register */
    USART_SendData(USART1, TxBuffer[TxCount++]);
    if(TxCount == TxLength)
    {
      /* Disable the EVAL_COM1 Transmit interrupt */
      USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
    }
  }
}

void USART2_IRQHandler(void)
{
  if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
  {
    /* Read one byte from the receive data register */
    RxBuffer2[RxHead2++] = (USART_ReceiveData(USART2) & 0x7F);
		counter2++;
    if(RxHead2 >= MAVLINK)
    {
      /* Disable the EVAL_COM1 Receive interrupt */
      RxHead2 = 0;
    }
		if(RxHead2 == RxEnd2){
			RxHead2 = 0;
			RxEnd2 = 0;
		}
  }

  if(USART_GetITStatus(USART2, USART_IT_TXE) != RESET)
  {   
    /* Write one byte to the transmit data register */
    USART_SendData(USART2, TxBuffer2[TxCount2++]);

    if(TxCount2 == TxLength2)
    {
      /* Disable the EVAL_COM1 Transmit interrupt */
      USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
    }
  }
}
/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
