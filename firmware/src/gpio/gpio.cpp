#include "firmware/src/gpio/gpio.hpp"

#include <cstdint>

#include <board.h>
#include <hpm_gpio_drv.h>
#include <hpm_gpiom_drv.h>

#include "firmware/src/spi/bmi088/accel.hpp"
#include "firmware/src/spi/bmi088/gyro.hpp"

namespace {

constexpr uint8_t BMI088_INT_GYRO_PIN = 15; // PB15, active-low
constexpr uint8_t BMI088_INT_ACCEL_PIN = 0; // PY00, active-low

void configure_bmi088_int_gyro() {
    HPM_IOC->PAD[IOC_PAD_PB15].FUNC_CTL = IOC_PB15_FUNC_CTL_GPIO_B_15;
    gpiom_set_pin_controller(HPM_GPIOM, GPIOM_ASSIGN_GPIOB, BMI088_INT_GYRO_PIN, gpiom_soc_gpio0);
    gpio_set_pin_input(HPM_GPIO0, GPIO_OE_GPIOB, BMI088_INT_GYRO_PIN);

    gpio_config_pin_interrupt(
        HPM_GPIO0, GPIO_DI_GPIOB, BMI088_INT_GYRO_PIN, gpio_interrupt_trigger_edge_falling);
    gpio_clear_pin_interrupt_flag(HPM_GPIO0, GPIO_IF_GPIOB, BMI088_INT_GYRO_PIN);
    gpio_enable_pin_interrupt(HPM_GPIO0, GPIO_IE_GPIOB, BMI088_INT_GYRO_PIN);
    intc_m_enable_irq_with_priority(IRQn_GPIO0_B, 1);
}

void configure_bmi088_int_accel() {
    HPM_IOC->PAD[IOC_PAD_PY00].FUNC_CTL = IOC_PY00_FUNC_CTL_GPIO_Y_00;
    HPM_PIOC->PAD[IOC_PAD_PY00].FUNC_CTL = PIOC_PY00_FUNC_CTL_SOC_GPIO_Y_00;
    gpiom_set_pin_controller(HPM_GPIOM, GPIOM_ASSIGN_GPIOY, BMI088_INT_ACCEL_PIN, gpiom_soc_gpio0);
    gpio_set_pin_input(HPM_GPIO0, GPIO_OE_GPIOY, BMI088_INT_ACCEL_PIN);

    gpio_config_pin_interrupt(
        HPM_GPIO0, GPIO_DI_GPIOY, BMI088_INT_ACCEL_PIN, gpio_interrupt_trigger_edge_falling);
    gpio_clear_pin_interrupt_flag(HPM_GPIO0, GPIO_IF_GPIOY, BMI088_INT_ACCEL_PIN);
    gpio_enable_pin_interrupt(HPM_GPIO0, GPIO_IE_GPIOY, BMI088_INT_ACCEL_PIN);
    intc_m_enable_irq_with_priority(IRQn_GPIO0_Y, 1);
}

} // namespace

namespace librmcs::firmware::gpio {

void init_bmi088_interrupts() {
    configure_bmi088_int_gyro();
    configure_bmi088_int_accel();
}

SDK_DECLARE_EXT_ISR_M(IRQn_GPIO0_B, gpio_bmi088_int_gyro_isr)
void gpio_bmi088_int_gyro_isr() {
    if (gpio_check_clear_interrupt_flag(HPM_GPIO0, GPIO_DI_GPIOB, BMI088_INT_GYRO_PIN)) {
        spi::bmi088::gyroscope->data_ready_callback();
    }
}

SDK_DECLARE_EXT_ISR_M(IRQn_GPIO0_Y, gpio_bmi088_int_accel_isr)
void gpio_bmi088_int_accel_isr() {
    if (gpio_check_clear_interrupt_flag(HPM_GPIO0, GPIO_DI_GPIOY, BMI088_INT_ACCEL_PIN)) {
        spi::bmi088::accelerometer->data_ready_callback();
    }
}

} // namespace librmcs::firmware::gpio
