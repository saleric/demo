/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  *
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"

typedef struct Bits_t
{
    struct BitSet_t
    {
        uint8_t Bit0  :1;
        uint8_t Bit1  :1;
        uint8_t Bit2  :1;
        uint8_t Bit3  :1;
        uint8_t Bit4  :1;
        uint8_t Bit5  :1;
        uint8_t Bit6  :1;
        uint8_t Bit7  :1;
        uint8_t Bit8  :1;
        uint8_t Bit9  :1;
        uint8_t Bit10 :1;
        uint8_t Bit11 :1;
        uint8_t Bit12 :1;
        uint8_t Bit13 :1;
        uint8_t Bit14 :1;
        uint8_t Bit15 :1;
    } BitSet;
    
    struct BitReset_t
    {
        uint8_t Bit0  :1;
        uint8_t Bit1  :1;
        uint8_t Bit2  :1;
        uint8_t Bit3  :1;
        uint8_t Bit4  :1;
        uint8_t Bit5  :1;
        uint8_t Bit6  :1;
        uint8_t Bit7  :1;
        uint8_t Bit8  :1;
        uint8_t Bit9  :1;
        uint8_t Bit10 :1;
        uint8_t Bit11 :1;
        uint8_t Bit12 :1;
        uint8_t Bit13 :1;
        uint8_t Bit14 :1;
        uint8_t Bit15 :1;
    } BitReset;
    
}Bits;

#define PortDBits (*((volatile Bits*)&(GPIOD->BSRR)))

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

extern volatile uint32_t sampleReceived;
extern volatile uint32_t spiSendReady;
       volatile uint32_t ledToggle;
       volatile uint32_t spiCnt;

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles TIM2 global interrupt.
*/
void TIM2_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim2);
    if (ledToggle) {
        PortDBits.BitReset.Bit12 = 1;
        ledToggle = 0;
    }
    else {
        PortDBits.BitSet.Bit12 = 1;
        ledToggle = 1;
    }
}

/**
* @brief This function handles TIM3 global interrupt.
*/
void TIM3_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim3);
}

/**
* @brief This function handles TIM4 global interrupt.
*/
void TIM4_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim4);
    if (ledToggle) {
        PortDBits.BitReset.Bit14 = 1;
        ledToggle = 0;
    }
    else {
        PortDBits.BitSet.Bit14 = 1;
        ledToggle = 1;
    }
    
    if (spiCnt == 19) {
        spiSendReady = 1;
        spiCnt = 0;
    }   
    else {
        spiCnt++;
    }
}

/**
* @brief This function handles DMA2 stream0 global interrupt.
*/
void DMA2_Stream0_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_adc1);
    sampleReceived = 1;
}

/*****************************END OF FILE****/
