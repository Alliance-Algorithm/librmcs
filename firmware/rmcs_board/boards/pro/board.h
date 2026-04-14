#pragma once

#include <stdint.h>

#include <hpm_common.h>
#include <hpm_gpio_regs.h>
#include <hpm_soc.h>
#include <hpm_usb_regs.h>

#define BOARD_NAME                     "rmcs_board"
#define BOARD_UF2_SIGNATURE            (0x0A4D5048UL)
#define BOARD_FLASH_BASE_ADDRESS       (0x80000000UL)
#define BOARD_FLASH_SIZE               (SIZE_1MB)
#define BOARD_APP_XPI_NOR_XPI_BASE     (HPM_XPI0)
#define BOARD_APP_XPI_NOR_CFG_OPT_HDR  (0xfcf90002U)
#define BOARD_APP_XPI_NOR_CFG_OPT_OPT0 (0x00000005U)
#define BOARD_APP_XPI_NOR_CFG_OPT_OPT1 (0x00001000U)

/* User LED / Button */
#define BOARD_LED_GPIO_CTRL  HPM_GPIO0
#define BOARD_LED_GPIO_INDEX GPIO_DI_GPIOA
#define BOARD_LED_GPIO_PIN   23
#define BOARD_LED_OFF_LEVEL  1
#define BOARD_LED_ON_LEVEL   0

#define BOARD_APP_GPIO_CTRL  HPM_GPIO0
#define BOARD_APP_GPIO_INDEX GPIO_DI_GPIOA
#define BOARD_APP_GPIO_PIN   9

#ifdef __cplusplus
extern "C" {
#endif

void board_init(void);
void board_init_usb(void);

void board_delay_us(uint32_t us);
void board_delay_ms(uint32_t ms);

#ifdef __cplusplus
}
#endif
