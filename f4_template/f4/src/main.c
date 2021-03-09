/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "version.h"
#include "commands.h"
#include "hal.h"
#include "usb_device.h"


void SystemClock_Config(void);

uint32_t systick_freq;
volatile uint64_t systime = 0;

#define xstr(s) str(s)
#define str(s) #s

volatile const fw_info_t fw_info __attribute__((section(".version_info"))) = {
  .product_name = "STMLV",
  .git_version = xstr(GIT_HASH),
  .git_branch = xstr(GIT_BRANCH),
  .build_user = xstr(USR),
  .build_host = xstr(HOST),
  .build_date = __DATE__,
  .build_time = __TIME__,
  .build_cc = xstr(BUILD_CC),
  .cc_version = xstr(BUILD_CC_VERSION),
  .major        = VERSION_MAJOR,
  .minor        = VERSION_MINOR,
  .patch        = VERSION_PATCH,
};

extern volatile const uint32_t bin_crc;
extern volatile const uint32_t bin_size;

void about(char *ptr) {
  // bin_info_t *bin_info = &fw_info - sizeof(bin_info_t) / 4;
  printf("*** BUILD INFO ***\n");
  printf("crc: %u\n", bin_crc);
  printf("size: %u\n", bin_size);
  
  printf("name: %s\n", fw_info.product_name);
  printf("git hash: %s\n", fw_info.git_version);
  printf("git branch: %s\n", fw_info.git_branch);
  printf("version: %u.%u.%u\n", fw_info.major, fw_info.minor, fw_info.patch);

  printf("usr: %s\n", fw_info.build_user);
  printf("host: %s\n", fw_info.build_host);
  printf("date: %s\n", fw_info.build_date);
  printf("time: %s\n", fw_info.build_time);

  printf("cc: %s\n", fw_info.build_cc);
  printf("cc version: %s\n", fw_info.cc_version);
}
COMMAND("about", about, "print build info");

void SysTick_Handler(void) {
  HAL_IncTick();

  systime++;
  hal_run_rt();
}

uint32_t hal_get_systick_value() {
  return (SysTick->VAL);
}

uint32_t hal_get_systick_reload() {
  return (SysTick->LOAD);
}

uint32_t hal_get_systick_freq() {
  return (systick_freq);
}

void bootloader(char *ptr) {
  hal_stop();

  RCC->APB1ENR |= RCC_APB1ENR_PWREN; // en power block
  PWR->CR |= PWR_CR_DBP; // disable write protection
  RCC->BDCR |= 0x10 << RCC_BDCR_RTCSEL_Pos; // LSI -> RTC
  RCC->BDCR |= RCC_BDCR_RTCEN; // LSI -> RTC

  RTC->BKP0R = 0xDEADBEEF; // dfu trigger
  HAL_NVIC_SystemReset();
}
COMMAND("bootloader", bootloader, "enter bootloader");

void reset(char *ptr) {
  hal_stop();

  GPIOA->MODER &= ~GPIO_MODER_MODER12_Msk;
  GPIOA->MODER |= GPIO_MODER_MODER12_0;
  GPIOA->ODR &= ~GPIO_PIN_12;
  HAL_Delay(100);

  HAL_NVIC_SystemReset();
}
COMMAND("reset", reset, "reset STMLV");


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  extern void *g_pfnVectors;
  SCB->VTOR = (uint32_t)&g_pfnVectors;

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();
  systick_freq = HAL_RCC_GetHCLKFreq();
  __enable_irq();

  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_GPIOEEN | RCC_AHB1ENR_GPIOFEN | RCC_AHB1ENR_GPIOGEN | RCC_AHB1ENR_GPIOHEN;


  /* Initialize all configured peripherals */
  MX_USB_DEVICE_Init();

  hal_init(1.0 / 5000.0, 0.0);
  hal_parse("load term");
  hal_parse("term0.rt_prio = 10");

  hal_parse("flashloadconf");
  hal_parse("loadconf");

  hal_parse("start");
  
  while (1)
  {
    cdc_poll();
    hal_run_nrt(); 
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }

  // HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);
  // HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
  // HAL_NVIC_EnableIRQ(SysTick_IRQn);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
