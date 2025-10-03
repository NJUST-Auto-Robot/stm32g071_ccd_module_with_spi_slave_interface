/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "pid.h"
pid_t explose_pid;
uint32_t pixel_sum;
#define USE_DATA_NUM 1000
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t calc_raw_max_dummy(void);

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
enum E_state {
    E_wait_sh_raising,
    E_wait_icg_raising,
    E_wait_dma_ok,
    // E_wait_dma_ok,
};
__IO enum E_state cur_state;
uint16_t adc_data[1546];
uint16_t adc_data_static[USE_DATA_NUM];
uint8_t adc_data_static_u8[USE_DATA_NUM];
__IO uint8_t ccd_frame_ok = 0;
int32_t explose_time_us = 2000;
void ccd_start_icg(void);
__IO uint8_t ccd_want_update_sh_v = 0;
void ccd_want_update_sh(void)
{
    ccd_want_update_sh_v = 1;
    // LL_TIM_ClearFlag_UPDATE(TIM2);
    // LL_TIM_EnableIT_UPDATE(TIM2);
}
void ccd_update_sh(void)
{
    // LL_TIM_DisableIT_UPDATE(TIM2);
    if (explose_time_us > 10 && TIM2->ARR != explose_time_us)
    {
        // LL_TIM_DisableCounter(TIM2);
        TIM2->ARR = explose_time_us;
        TIM2->CCR2 = TIM2->ARR - 3;
        if (TIM2->CNT > TIM2->CCR2 - 100)
            TIM2->CNT = TIM2->CCR2 - 100;
        // LL_TIM_GenerateEvent_UPDATE(TIM2);
    // LL_TIM_EnableCounter(TIM2);
    }
}
void ccd_waiting_sh(void)
{
    // if (explose_time_us > 10 && TIM2->ARR != explose_time_us)
    // {
    //     LL_TIM_DisableCounter(TIM2);
    //     TIM2->ARR = explose_time_us;
    //     TIM2->CCR2 = TIM2->ARR - 2;
    //     LL_TIM_GenerateEvent_UPDATE(TIM2);
    //     LL_TIM_EnableCounter(TIM2);
    // }
    LL_TIM_ClearFlag_CC2(TIM2);
    LL_TIM_EnableIT_CC2(TIM2);
    cur_state = E_wait_sh_raising;
}
void ccd_sh_want_raise(void)
{
    LL_TIM_ClearFlag_CC2(TIM2);
    LL_TIM_DisableIT_CC2(TIM2);
    if (TIM2->CNT < TIM2->CCR2)
    {
        // __IO uint8_t x;
        // (void)x;
        ccd_waiting_sh();
    }
    else if (E_wait_sh_raising == cur_state)
    {
        ccd_start_icg();
        cur_state = E_wait_icg_raising;
    }
}
void ccd_start_icg(void)
{
    TIM16->PSC = 31;
    TIM16->ARR = 20 * 2 - 1;
    TIM16->CCR1 = 1;
    LL_TIM_EnableAllOutputs(TIM16);
    LL_TIM_CC_EnableChannel(TIM16, LL_TIM_CHANNEL_CH1);
    LL_TIM_ClearFlag_UPDATE(TIM16);
    LL_TIM_EnableIT_UPDATE(TIM16);
    LL_TIM_SetOnePulseMode(TIM16, LL_TIM_ONEPULSEMODE_SINGLE);
    LL_TIM_EnableCounter(TIM16);
}
void ccd_end_icg(void)
{
    // LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_6);
    LL_TIM_ClearFlag_UPDATE(TIM16);
    LL_TIM_DisableIT_UPDATE(TIM16);
    LL_TIM_DisableCounter(TIM16);
    if (E_wait_icg_raising == cur_state)
    {
        LL_ADC_ClearFlag_OVR(ADC1);
        LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t)adc_data);
        LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_1, LL_ADC_DMA_GetRegAddr(ADC1, LL_ADC_DMA_REG_REGULAR_DATA));
        LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, sizeof(adc_data) / sizeof(adc_data[0]));
        LL_DMA_ClearFlag_HT1(DMA1);
        LL_DMA_ClearFlag_TE1(DMA1);
        LL_DMA_ClearFlag_TC1(DMA1);
        LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_1);
        LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
        LL_ADC_REG_StartConversion(ADC1);
        cur_state = E_wait_dma_ok;
    }
}
void ccd_end_transfer(void)
{
    static int is_first = 0;
    LL_ADC_REG_StopConversion(ADC1);
    // LL_ADC_Disable(ADC1);
    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_1);
    LL_DMA_ClearFlag_HT1(DMA1);
    LL_DMA_ClearFlag_TE1(DMA1);
    LL_DMA_ClearFlag_TC1(DMA1);
    if (E_wait_dma_ok == cur_state)
    {
        if (is_first < 5)
        {
            is_first++;
            ccd_waiting_sh();
        }
        else
        {
            if (ccd_want_update_sh_v)
            {
                ccd_want_update_sh_v = 0;
                ccd_update_sh();
            }
            uint16_t max = calc_raw_max_dummy();
            pixel_sum = 0;
            // int err_cnt = 0;
            // for (int i = 0;i < USE_DATA_NUM;i++)
            // {
            //     if (((max - adc_data[i + 32 + (1500 - USE_DATA_NUM) / 2]) < 1))
            //     {
            //         err_cnt++;
            //     }
            // }
            for (int i = 0;i < USE_DATA_NUM;i++)
            {
                adc_data_static_u8[i] = ((max - adc_data[i + 32 + (1500 - USE_DATA_NUM) / 2])) / 10;
                pixel_sum += adc_data_static_u8[i];
            }
            // if (err_cnt > 10)
            // {
            // }
            // else
            // {
            ccd_frame_ok = 1;
            ccd_waiting_sh();
            // }
        }
    }
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void spi_trams(uint8_t* d, uint16_t len)
{
    for (int i = 0;i < len;i++)
    {
        LL_SPI_TransmitData8(SPI2, d[i]);
        while (!LL_SPI_IsActiveFlag_TXE(SPI2))
            ;
    }
}
uint16_t calc_raw_max_dummy(void)
{
    uint32_t sum = 0;;
    for (int i = 0;i < 14;i++)
    {
        sum += adc_data[i];
    }
    return sum / 14;
}
void pid_calc_timely(void)
{
    static int last = 2000;
    if (pixel_sum)
    {
        int exp = pid_calc(&explose_pid, pixel_sum, USE_DATA_NUM * 120);
        last += exp;
        if (last < 200)
            last = 300;
        if (last > 150000)
            last = 150000;
        // if (explose_time_us - last > 50 || explose_time_us - last < -50)
        {
            // explose_time_us = last / 100 * 100;
            explose_time_us = last;
            ccd_want_update_sh();
        }
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, 3);

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_SPI2_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM17_Init();
  MX_TIM16_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */

                                          // \phi m clock 2Mhz
    TIM3->PSC = 0;
    TIM3->ARR = 31;
    TIM3->CCR2 = 15;
    LL_TIM_GenerateEvent_UPDATE(TIM3);
    LL_TIM_EnableAllOutputs(TIM3);
    LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH2);

    // adc convert clock 1Mhz
    TIM6->PSC = 0;
    TIM6->ARR = 63;
    LL_TIM_GenerateEvent_UPDATE(TIM6);
    LL_TIM_EnableCounter(TIM6);

    LL_TIM_EnableCounter(TIM3);
    // sh  high level>1us, tint>10us
    TIM2->PSC = 63;
    TIM2->ARR = 999;
    TIM2->CCR1 = 3;
    TIM2->CCR2 = TIM2->ARR - 4;
    LL_TIM_GenerateEvent_UPDATE(TIM2);
    LL_TIM_EnableAllOutputs(TIM2);
    LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH1);
    LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH2);
    LL_TIM_EnableCounter(TIM2);

    LL_ADC_StartCalibration(ADC1);
    while (LL_ADC_IsCalibrationOnGoing(ADC1));
    LL_ADC_Enable(ADC1);
    // LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_8);

    LL_SPI_Enable(SPI2);

    PID_struct_init(&explose_pid, DELTA_PID, 100000, 100000, 0.01, 0, 0);
    TIM17->PSC = 63999;
    TIM17->ARR = 50;
    LL_TIM_EnableIT_UPDATE(TIM17);
    LL_TIM_EnableCounter(TIM17);
    //LL_ADC_REG_StartConversion(ADC1);
    // LL_mDelay(1);
    ccd_waiting_sh();
    // ccd_start_icg();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
        if (ccd_frame_ok)
        {
            ccd_frame_ok = 0;
            uint8_t d[8] = { 0x3d, 0x7e ,0,0,1,0, 0x7e, 0x3d };
            *(uint16_t*)(d + 2) = USE_DATA_NUM;
            LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_8);
            //LL_mDelay(1);
            spi_trams(d, 8);
            if (d[4])
                spi_trams((uint8_t*)adc_data_static_u8, *(uint16_t*)(d + 2));
            else
            {
                spi_trams((uint8_t*)&adc_data[32 + (1500 - USE_DATA_NUM) / 2], (*(uint16_t*)(d + 2)) * 2);
            }
            LL_mDelay(1);
            LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_8);
            LL_mDelay(1);
        }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
  while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_2)
  {
  }

  /* HSI configuration and activation */
  LL_RCC_HSI_Enable();
  while(LL_RCC_HSI_IsReady() != 1)
  {
  }

  /* Main PLL configuration and activation */
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLLM_DIV_1, 8, LL_RCC_PLLR_DIV_2);
  LL_RCC_PLL_Enable();
  LL_RCC_PLL_EnableDomain_SYS();
  while(LL_RCC_PLL_IsReady() != 1)
  {
  }

  /* Set AHB prescaler*/
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);

  /* Sysclk activation on the main PLL */
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  }

  /* Set APB1 prescaler*/
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_Init1msTick(64000000);
  /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
  LL_SetSystemCoreClock(64000000);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
                                            /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1)
    {
    }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
                                            /* User can add his own implementation to report the file name and line number,
                                               ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
