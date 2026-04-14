#pragma once

#include <stdint.h>

#include <hpm_common.h>
#include <hpm_soc.h>
#include <hpm_usb_regs.h>

#define BOARD_NAME                     "RMCS_Slave_Lite"
#define BOARD_UF2_SIGNATURE            (0x0A4D5048UL)
#define BOARD_FLASH_BASE_ADDRESS       (0x80000000UL)
#define BOARD_FLASH_SIZE               (SIZE_1MB)
#define BOARD_APP_XPI_NOR_XPI_BASE     (HPM_XPI0)
#define BOARD_APP_XPI_NOR_CFG_OPT_HDR  (0xfcf90002U)
#define BOARD_APP_XPI_NOR_CFG_OPT_OPT0 (0x00000005U)
#define BOARD_APP_XPI_NOR_CFG_OPT_OPT1 (0x00001000U)

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
