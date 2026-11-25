#include "board.h"

#include <stdint.h>

#include <hpm_clock_drv.h>
#include <hpm_common.h>
#include <hpm_gpio_drv.h>
#include <hpm_gpio_regs.h>
#include <hpm_gpiom_drv.h>
#include <hpm_gpiom_regs.h>
#include <hpm_gpiom_soc_drv.h>
#include <hpm_ioc_regs.h>
#include <hpm_iomux.h>
#include <hpm_l1c_drv.h>
#include <hpm_pllctlv2_drv.h>
#include <hpm_pmp_drv.h>
#include <hpm_soc.h>
#include <hpm_soc_feature.h>
#include <hpm_sysctl_drv.h>
#include <hpm_usb_drv.h>

#if defined(FLASH_XIP) && FLASH_XIP
__attribute__((section(".nor_cfg_option"), used)) const uint32_t kOption[] = {
    BOARD_APP_XPI_NOR_CFG_OPT_HDR, BOARD_APP_XPI_NOR_CFG_OPT_OPT0, BOARD_APP_XPI_NOR_CFG_OPT_OPT1,
    0};
#endif

#if defined(FLASH_UF2) && FLASH_UF2
ATTR_PLACE_AT(".uf2_signature")
__attribute__((used)) const uint32_t kUf2Signature = BOARD_UF2_SIGNATURE;
#endif

void board_init(void) {
    if (clock_get_frequency(clock_cpu0) == PLLCTL_SOC_PLL_REFCLK_FREQ) {
        pllctlv2_xtal_set_rampup_time(HPM_PLLCTLV2, 32UL * 1000UL * 9U);
        sysctl_clock_set_preset(HPM_SYSCTL, 2);
    }

    clock_add_to_group(clock_cpu0, 0);
    clock_add_to_group(clock_ahb0, 0);
    clock_add_to_group(clock_axif, 0);
    clock_add_to_group(clock_axis, 0);
    clock_add_to_group(clock_axic, 0);
    clock_add_to_group(clock_lmm0, 0);
    clock_add_to_group(clock_ram0, 0);
    clock_add_to_group(clock_rom0, 0);
    clock_add_to_group(clock_xpi0, 0);
    clock_add_to_group(clock_gpio, 0);
#if defined(FLASH_UF2) && FLASH_UF2
    clock_add_to_group(clock_hdma, 0);
    clock_add_to_group(clock_xdma, 0);
#endif
    clock_add_to_group(clock_mchtmr0, 0);
    clock_connect_group_to_cpu(0, 0);

    clock_update_core_clock();
    clock_set_source_divider(clock_mchtmr0, clk_src_osc24m, 6);

#if defined(FLASH_UF2) && FLASH_UF2
    // The linker aligns this AXI SRAM region for NAPOT; USB DMA needs it uncached.
    // NOLINTNEXTLINE(bugprone-reserved-identifier, readability-identifier-naming)
    extern uint8_t __noncacheable_start__[], __noncacheable_end__[];
    const uintptr_t dma_start = (uintptr_t)__noncacheable_start__;
    const uint32_t dma_size = (uint32_t)((uintptr_t)__noncacheable_end__ - dma_start);
    if (l1c_dc_is_enabled()) {
        l1c_dc_flush(dma_start, dma_size);
    }
    const pma_attr_t dma_region = {
        .pma_cfg.val = PMA_CFG(ADDR_MATCH_NAPOT, MEM_TYPE_MEM_NON_CACHE_BUF, AMO_EN),
        .pma_addr = PMA_NAPOT_ADDR(dma_start, dma_size),
    };
    (void)pma_config_attributes(&dma_region, 1U);
#endif
}

bool board_check_bootloader_force_stay_requested(void) {
    clock_add_to_group(clock_gpio, 0);
    gpiom_set_pin_controller(HPM_GPIOM, GPIOM_ASSIGN_GPIOF, 26, gpiom_soc_gpio0);
    gpio_set_pin_input(HPM_GPIO0, GPIO_OE_GPIOF, 26);
    HPM_IOC->PAD[IOC_PAD_PF26].FUNC_CTL = IOC_PF26_FUNC_CTL_GPIO_F_26;
    HPM_IOC->PAD[IOC_PAD_PF26].PAD_CTL = IOC_PAD_PAD_CTL_HYS_SET(1);
    for (uint32_t i = 0; i < 4; ++i) {
        board_delay_us(250);
        if (gpio_read_pin(HPM_GPIO0, GPIO_DI_GPIOF, 26))
            return false;
    }
    return true;
}

void board_init_usb(void) {
    clock_add_to_group(clock_usb0, 0);
    usb_phy_using_external_vbus(HPM_USB0);
}

void board_delay_us(uint32_t us) { clock_cpu_delay_us(us); }

void board_delay_ms(uint32_t ms) { clock_cpu_delay_ms(ms); }
