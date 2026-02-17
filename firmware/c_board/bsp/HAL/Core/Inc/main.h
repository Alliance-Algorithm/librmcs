/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "stm32f4xx_hal.h"     // IWYU pragma: export
#include "stm32f4xx_hal_def.h" // IWYU pragma: export
#include "stm32f407xx.h"       // IWYU pragma: export

#ifdef HAL_RCC_MODULE_ENABLED
# include "stm32f4xx_hal_rcc.h" // IWYU pragma: export
#endif

#ifdef HAL_GPIO_MODULE_ENABLED
# include "stm32f4xx_hal_gpio.h" // IWYU pragma: export
#endif

#ifdef HAL_EXTI_MODULE_ENABLED
# include "stm32f4xx_hal_exti.h" // IWYU pragma: export
#endif

#ifdef HAL_DMA_MODULE_ENABLED
# include "stm32f4xx_hal_dma.h" // IWYU pragma: export
#endif

#ifdef HAL_CORTEX_MODULE_ENABLED
# include "stm32f4xx_hal_cortex.h" // IWYU pragma: export
#endif

#ifdef HAL_ADC_MODULE_ENABLED
# include "stm32f4xx_hal_adc.h" // IWYU pragma: export
#endif

#ifdef HAL_CAN_MODULE_ENABLED
# include "stm32f4xx_hal_can.h" // IWYU pragma: export
#endif

#ifdef HAL_CAN_LEGACY_MODULE_ENABLED
# include "stm32f4xx_hal_can_legacy.h" // IWYU pragma: export
#endif

#ifdef HAL_CRC_MODULE_ENABLED
# include "stm32f4xx_hal_crc.h" // IWYU pragma: export
#endif

#ifdef HAL_CRYP_MODULE_ENABLED
# include "stm32f4xx_hal_cryp.h" // IWYU pragma: export
#endif

#ifdef HAL_DMA2D_MODULE_ENABLED
# include "stm32f4xx_hal_dma2d.h" // IWYU pragma: export
#endif

#ifdef HAL_DAC_MODULE_ENABLED
# include "stm32f4xx_hal_dac.h" // IWYU pragma: export
#endif

#ifdef HAL_DCMI_MODULE_ENABLED
# include "stm32f4xx_hal_dcmi.h" // IWYU pragma: export
#endif

#ifdef HAL_ETH_MODULE_ENABLED
# include "stm32f4xx_hal_eth.h" // IWYU pragma: export
#endif

#ifdef HAL_ETH_LEGACY_MODULE_ENABLED
# include "stm32f4xx_hal_eth_legacy.h" // IWYU pragma: export
#endif

#ifdef HAL_FLASH_MODULE_ENABLED
# include "stm32f4xx_hal_flash.h" // IWYU pragma: export
#endif

#ifdef HAL_SRAM_MODULE_ENABLED
# include "stm32f4xx_hal_sram.h" // IWYU pragma: export
#endif

#ifdef HAL_NOR_MODULE_ENABLED
# include "stm32f4xx_hal_nor.h" // IWYU pragma: export
#endif

#ifdef HAL_NAND_MODULE_ENABLED
# include "stm32f4xx_hal_nand.h" // IWYU pragma: export
#endif

#ifdef HAL_PCCARD_MODULE_ENABLED
# include "stm32f4xx_hal_pccard.h" // IWYU pragma: export
#endif

#ifdef HAL_SDRAM_MODULE_ENABLED
# include "stm32f4xx_hal_sdram.h" // IWYU pragma: export
#endif

#ifdef HAL_HASH_MODULE_ENABLED
# include "stm32f4xx_hal_hash.h" // IWYU pragma: export
#endif

#ifdef HAL_I2C_MODULE_ENABLED
# include "stm32f4xx_hal_i2c.h" // IWYU pragma: export
#endif

#ifdef HAL_SMBUS_MODULE_ENABLED
# include "stm32f4xx_hal_smbus.h" // IWYU pragma: export
#endif

#ifdef HAL_I2S_MODULE_ENABLED
# include "stm32f4xx_hal_i2s.h" // IWYU pragma: export
#endif

#ifdef HAL_IWDG_MODULE_ENABLED
# include "stm32f4xx_hal_iwdg.h" // IWYU pragma: export
#endif

#ifdef HAL_LTDC_MODULE_ENABLED
# include "stm32f4xx_hal_ltdc.h" // IWYU pragma: export
#endif

#ifdef HAL_PWR_MODULE_ENABLED
# include "stm32f4xx_hal_pwr.h" // IWYU pragma: export
#endif

#ifdef HAL_RNG_MODULE_ENABLED
# include "stm32f4xx_hal_rng.h" // IWYU pragma: export
#endif

#ifdef HAL_RTC_MODULE_ENABLED
# include "stm32f4xx_hal_rtc.h" // IWYU pragma: export
#endif

#ifdef HAL_SAI_MODULE_ENABLED
# include "stm32f4xx_hal_sai.h" // IWYU pragma: export
#endif

#ifdef HAL_SD_MODULE_ENABLED
# include "stm32f4xx_hal_sd.h" // IWYU pragma: export
#endif

#ifdef HAL_SPI_MODULE_ENABLED
# include "stm32f4xx_hal_spi.h" // IWYU pragma: export
#endif

#ifdef HAL_TIM_MODULE_ENABLED
# include "stm32f4xx_hal_tim.h" // IWYU pragma: export
#endif

#ifdef HAL_UART_MODULE_ENABLED
# include "stm32f4xx_hal_uart.h" // IWYU pragma: export
#endif

#ifdef HAL_USART_MODULE_ENABLED
# include "stm32f4xx_hal_usart.h" // IWYU pragma: export
#endif

#ifdef HAL_IRDA_MODULE_ENABLED
# include "stm32f4xx_hal_irda.h" // IWYU pragma: export
#endif

#ifdef HAL_SMARTCARD_MODULE_ENABLED
# include "stm32f4xx_hal_smartcard.h" // IWYU pragma: export
#endif

#ifdef HAL_WWDG_MODULE_ENABLED
# include "stm32f4xx_hal_wwdg.h" // IWYU pragma: export
#endif

#ifdef HAL_PCD_MODULE_ENABLED
# include "stm32f4xx_hal_pcd.h" // IWYU pragma: export
#endif

#ifdef HAL_HCD_MODULE_ENABLED
# include "stm32f4xx_hal_hcd.h" // IWYU pragma: export
#endif

#ifdef HAL_DSI_MODULE_ENABLED
# include "stm32f4xx_hal_dsi.h" // IWYU pragma: export
#endif

#ifdef HAL_QSPI_MODULE_ENABLED
# include "stm32f4xx_hal_qspi.h" // IWYU pragma: export
#endif

#ifdef HAL_CEC_MODULE_ENABLED
# include "stm32f4xx_hal_cec.h" // IWYU pragma: export
#endif

#ifdef HAL_FMPI2C_MODULE_ENABLED
# include "stm32f4xx_hal_fmpi2c.h" // IWYU pragma: export
#endif

#ifdef HAL_FMPSMBUS_MODULE_ENABLED
# include "stm32f4xx_hal_fmpsmbus.h" // IWYU pragma: export
#endif

#ifdef HAL_SPDIFRX_MODULE_ENABLED
# include "stm32f4xx_hal_spdifrx.h" // IWYU pragma: export
#endif

#ifdef HAL_DFSDM_MODULE_ENABLED
# include "stm32f4xx_hal_dfsdm.h" // IWYU pragma: export
#endif

#ifdef HAL_LPTIM_MODULE_ENABLED
# include "stm32f4xx_hal_lptim.h" // IWYU pragma: export
#endif

#ifdef HAL_MMC_MODULE_ENABLED
# include "stm32f4xx_hal_mmc.h" // IWYU pragma: export
#endif

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void AppEntry(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SPI1_MISO_Pin       GPIO_PIN_4
#define SPI1_MISO_GPIO_Port GPIOB
#define SPI1_SCK_Pin        GPIO_PIN_3
#define SPI1_SCK_GPIO_Port  GPIOB
#define LED_R_Pin           GPIO_PIN_12
#define LED_R_GPIO_Port     GPIOH
#define LED_G_Pin           GPIO_PIN_11
#define LED_G_GPIO_Port     GPIOH
#define LED_B_Pin           GPIO_PIN_10
#define LED_B_GPIO_Port     GPIOH
#define CS1_ACCEL_Pin       GPIO_PIN_4
#define CS1_ACCEL_GPIO_Port GPIOA
#define INT1_ACC_Pin        GPIO_PIN_4
#define INT1_ACC_GPIO_Port  GPIOC
#define INT1_ACC_EXTI_IRQn  EXTI4_IRQn
#define INT1_GYRO_Pin       GPIO_PIN_5
#define INT1_GYRO_GPIO_Port GPIOC
#define INT1_GYRO_EXTI_IRQn EXTI9_5_IRQn
#define SPI1_MOSI_Pin       GPIO_PIN_7
#define SPI1_MOSI_GPIO_Port GPIOA
#define CS1_GYRO_Pin        GPIO_PIN_0
#define CS1_GYRO_GPIO_Port  GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
