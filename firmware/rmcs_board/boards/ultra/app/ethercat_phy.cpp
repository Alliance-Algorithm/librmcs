#include "ethercat_phy.hpp"

#include <cstdint>

#include <board.h>
#include <hpm_clock_drv.h>
#include <hpm_common.h>
#include <hpm_esc_drv.h>
#include <hpm_esc_regs.h>
#include <hpm_gpiom_soc_drv.h>
#include <hpm_gptmr_drv.h>
#include <hpm_ioc_regs.h>
#include <hpm_iomux.h>
#include <hpm_soc.h>

#include "firmware/rmcs_board/app/src/gpio/gpio_pin.hpp"

namespace librmcs::firmware::board {

namespace {

constexpr auto kPhyReset = make_gpio_pin<gpiom_soc_gpio0, 'E', 0, false>();
constexpr uint8_t kEcat0Address = 0;
constexpr uint8_t kEcat1Address = 1;
constexpr uint8_t kPhyPageSelect = 31;

} // namespace

bool EthercatPhy::start_clock() {
    kPhyReset.configure_controller();
    kPhyReset.set_active(true);
    kPhyReset.configure_as_output();
    kPhyReset.configure_ioc_function();

    clock_add_to_group(clock_esc0, 0);
    if (clock_set_source_divider(clock_esc0, clk_src_pll1_clk0, 4) != status_success
        || clock_get_frequency(clock_esc0) != 100'000'000) {
        return false;
    }

    configure_pins();
    HPM_ESC->GPR_CFG0 = HPM_ESC->GPR_CFG0 | ESC_GPR_CFG0_PORT2_DIS_MASK;
    HPM_ESC->PHY_CFG0 = ESC_PHY_CFG0_MAC_SPEED_SET(1);
    esc_eeprom_clock_config_t config{
        .eeprom_emulation = true,
        .eeprom_size_over_16kbit = false,
        .core_clock_en = true,
        .phy_refclk_en = false,
    };
    esc_config_eeprom_and_clock(HPM_ESC, &config);
    return start_reference_clock();
}

bool EthercatPhy::start_links() {
    // Apply the disconnected state after the ESC core clock and EEPROM are ready.
    esc_config_nmii_link_source(HPM_ESC, false, false, false);
    // Keep reset asserted while the common 25 MHz reference becomes stable.
    board_delay_ms(1);
    kPhyReset.set_active(false);
    board_delay_ms(15);

    esc_set_phy_offset(HPM_ESC, kEcat0Address);
    const bool phy1_present = has_phy(kEcat1Address);
    // PHY1 can answer at address zero until its broadcast response is disabled.
    const bool phy0_address_isolated = !phy1_present || disable_phy1_broadcast();
    const bool port0_ready =
        phy0_address_isolated && has_phy(kEcat0Address) && configure_phy(kEcat0Address);
    const bool port1_ready = phy1_present && configure_phy(kEcat1Address);
    esc_disable_pdi_access_mii_management(HPM_ESC);
    if (!port0_ready && !port1_ready) {
        kPhyReset.set_active(true);
        return false;
    }

    // LED1 is strapped low, so its asserted level is high. The ESC expects nMII_LINK.
    esc_config_ctrl_signal_function(HPM_ESC, 5, esc_ctrl_signal_func_alt_nmii_link0, true);
    esc_config_ctrl_signal_function(HPM_ESC, 2, esc_ctrl_signal_func_alt_nmii_link1, true);
    esc_config_nmii_link_source(HPM_ESC, port0_ready, port1_ready, false);
    return true;
}

bool EthercatPhy::start_reference_clock() {
    constexpr uint32_t timer_clock_hz = 100'000'000;
    constexpr uint32_t reference_clock_hz = 25'000'000;
    constexpr uint32_t period_ticks = timer_clock_hz / reference_clock_hz;

    clock_add_to_group(clock_gptmr2, 0);
    if (clock_set_source_divider(clock_gptmr2, clk_src_pll1_clk0, 4) != status_success
        || clock_get_frequency(clock_gptmr2) != timer_clock_hz) {
        return false;
    }

    gptmr_channel_config_t config;
    gptmr_channel_get_default_config(HPM_GPTMR2, &config);
    config.reload = period_ticks;
    config.cmp[0] = period_ticks / 2;
    config.debug_mode = false;
    if (gptmr_channel_config(HPM_GPTMR2, 0, &config, true) != status_success)
        return false;

    HPM_IOC->PAD[IOC_PAD_PA23].FUNC_CTL = IOC_PA23_FUNC_CTL_GPTMR2_COMP_0;
    return true;
}

bool EthercatPhy::has_phy(uint8_t address) {
    constexpr uint8_t id_register = 2;
    constexpr uint16_t jl1111_id = 0x937c;
    uint16_t id = 0;
    return esc_mdio_read(HPM_ESC, address, id_register, &id) == status_success && id == jl1111_id;
}

bool EthercatPhy::disable_phy1_broadcast() {
    constexpr uint16_t extended_page = 128;
    constexpr uint8_t address_config_register = 19;
    constexpr uint16_t phy1_address_config = (kEcat1Address << 5) | 0x1f;
    const auto write = [](uint8_t reg, uint16_t value) {
        return esc_mdio_write(HPM_ESC, kEcat1Address, reg, value) == status_success;
    };
    const bool broadcast_disabled =
        write(kPhyPageSelect, extended_page) && write(address_config_register, phy1_address_config);
    const bool page_restored = write(kPhyPageSelect, 0);
    return page_restored && broadcast_disabled;
}

bool EthercatPhy::configure_phy(uint8_t address) {
    constexpr uint8_t led_function_register = 19;
    constexpr uint16_t led_function_page = 7;
    constexpr uint16_t led1_link_only = 0;
    const auto write = [address](uint8_t reg, uint16_t value) {
        return esc_mdio_write(HPM_ESC, address, reg, value) == status_success;
    };
    // A blinking activity LED must not drive the ESC's physical link detection.
    const bool led_set =
        write(kPhyPageSelect, led_function_page) && write(led_function_register, led1_link_only);
    if (!write(kPhyPageSelect, 0) || !led_set)
        return false;

    constexpr uint8_t advertise_register = 4;
    constexpr uint16_t advertise_100base_tx_full_duplex = 0x0101;
    constexpr uint8_t control_register = 0;
    constexpr uint16_t restart_auto_negotiation = 0x1200;
    return write(advertise_register, advertise_100base_tx_full_duplex)
        && write(control_register, restart_auto_negotiation);
}

void EthercatPhy::configure_pins() {
    // PA09 is routed to ECAT0 RX_ER on the Ultra prototype wiring.
    HPM_IOC->PAD[IOC_PAD_PA09].FUNC_CTL = IOC_PA09_FUNC_CTL_GPIO_A_09;
    HPM_IOC->PAD[IOC_PAD_PA30].FUNC_CTL = IOC_PA30_FUNC_CTL_ESC0_MDIO;
    HPM_IOC->PAD[IOC_PAD_PA31].FUNC_CTL = IOC_PA31_FUNC_CTL_ESC0_MDC;
    HPM_IOC->PAD[IOC_PAD_PE01].FUNC_CTL = IOC_PE01_FUNC_CTL_ESC0_CTR_5;
    HPM_IOC->PAD[IOC_PAD_PE04].FUNC_CTL = IOC_PE04_FUNC_CTL_ESC0_CTR_2;

    HPM_IOC->PAD[IOC_PAD_PA16].FUNC_CTL = IOC_PA16_FUNC_CTL_ESC0_P0_RXDV;
    HPM_IOC->PAD[IOC_PAD_PA17].FUNC_CTL = IOC_PA17_FUNC_CTL_ESC0_P0_RXD_0;
    HPM_IOC->PAD[IOC_PAD_PA18].FUNC_CTL = IOC_PA18_FUNC_CTL_ESC0_P0_RXD_1;
    HPM_IOC->PAD[IOC_PAD_PA19].FUNC_CTL = IOC_PA19_FUNC_CTL_ESC0_P0_RXD_2;
    HPM_IOC->PAD[IOC_PAD_PA20].FUNC_CTL = IOC_PA20_FUNC_CTL_ESC0_P0_RXD_3;
    HPM_IOC->PAD[IOC_PAD_PA21].FUNC_CTL = IOC_PA21_FUNC_CTL_ESC0_P0_RXCK;
    HPM_IOC->PAD[IOC_PAD_PA24].FUNC_CTL = IOC_PA24_FUNC_CTL_ESC0_P0_TXCK;
    HPM_IOC->PAD[IOC_PAD_PA25].FUNC_CTL = IOC_PA25_FUNC_CTL_ESC0_P0_TXD_0;
    HPM_IOC->PAD[IOC_PAD_PA26].FUNC_CTL = IOC_PA26_FUNC_CTL_ESC0_P0_TXD_1;
    HPM_IOC->PAD[IOC_PAD_PA27].FUNC_CTL = IOC_PA27_FUNC_CTL_ESC0_P0_TXD_2;
    HPM_IOC->PAD[IOC_PAD_PA28].FUNC_CTL = IOC_PA28_FUNC_CTL_ESC0_P0_TXD_3;
    HPM_IOC->PAD[IOC_PAD_PA29].FUNC_CTL = IOC_PA29_FUNC_CTL_ESC0_P0_TXEN;

    HPM_IOC->PAD[IOC_PAD_PF02].FUNC_CTL = IOC_PF02_FUNC_CTL_ESC0_P1_TXCK;
    HPM_IOC->PAD[IOC_PAD_PF03].FUNC_CTL = IOC_PF03_FUNC_CTL_ESC0_P1_TXD_0;
    HPM_IOC->PAD[IOC_PAD_PF04].FUNC_CTL = IOC_PF04_FUNC_CTL_ESC0_P1_TXD_1;
    HPM_IOC->PAD[IOC_PAD_PF05].FUNC_CTL = IOC_PF05_FUNC_CTL_ESC0_P1_TXD_2;
    HPM_IOC->PAD[IOC_PAD_PF06].FUNC_CTL = IOC_PF06_FUNC_CTL_ESC0_P1_TXD_3;
    HPM_IOC->PAD[IOC_PAD_PF07].FUNC_CTL = IOC_PF07_FUNC_CTL_ESC0_P1_TXEN;
    HPM_IOC->PAD[IOC_PAD_PF08].FUNC_CTL = IOC_PF08_FUNC_CTL_ESC0_P1_RXDV;
    HPM_IOC->PAD[IOC_PAD_PF09].FUNC_CTL = IOC_PF09_FUNC_CTL_ESC0_P1_RXD_0;
    HPM_IOC->PAD[IOC_PAD_PF10].FUNC_CTL = IOC_PF10_FUNC_CTL_ESC0_P1_RXD_1;
    HPM_IOC->PAD[IOC_PAD_PF11].FUNC_CTL = IOC_PF11_FUNC_CTL_ESC0_P1_RXD_2;
    HPM_IOC->PAD[IOC_PAD_PF12].FUNC_CTL = IOC_PF12_FUNC_CTL_ESC0_P1_RXD_3;
    HPM_IOC->PAD[IOC_PAD_PF13].FUNC_CTL = IOC_PF13_FUNC_CTL_ESC0_P1_RXCK;
    HPM_IOC->PAD[IOC_PAD_PF14].FUNC_CTL = IOC_PF14_FUNC_CTL_ESC0_P1_RXER;
}

} // namespace librmcs::firmware::board
