#include "firmware/c_board/app/src/app.hpp"

#include <can.h>
#include <device/usbd.h>
#include <dma.h>
#include <gpio.h>
#include <main.h>
#include <spi.h>
#include <tim.h>
#include <usart.h>
#include <usb_otg.h>

#include "firmware/c_board/app/src/can/can.hpp"
#include "firmware/c_board/app/src/led/led.hpp"
#include "firmware/c_board/app/src/spi/bmi088/accel.hpp"
#include "firmware/c_board/app/src/spi/bmi088/gyro.hpp"
#include "firmware/c_board/app/src/spi/spi.hpp"
#include "firmware/c_board/app/src/uart/uart.hpp"
#include "firmware/c_board/app/src/usb/vendor.hpp"
#include "firmware/c_board/app/src/utility/interrupt_lock.hpp"

int main() { librmcs::firmware::app.init().run(); }

namespace librmcs::firmware {

App::App() {
    const utility::InterruptLockGuard guard;

    HAL_Init();
    SystemClock_Config();

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    MX_GPIO_Init();
    MX_DMA_Init();
    MX_SPI1_Init();
    MX_CAN1_Init();
    MX_CAN2_Init();
    MX_USART1_UART_Init();
    MX_USART3_UART_Init();
    MX_USART6_UART_Init();
    MX_TIM5_Init();
    MX_USB_OTG_FS_PCD_Init();

    led::led.init();
    usb::vendor.init();
    can::can1.init();
    can::can2.init();
    uart::uart1.init();
    uart::uart2.init();
    uart::uart_dbus.init();
    spi::bmi088::accelerometer.init();
    spi::bmi088::gyroscope.init();
}

// Non-static to ensure instantiation
// NOLINTNEXTLINE(readability-convert-member-functions-to-static)
[[noreturn]] void App::run() {
    while (true) {
        tud_task();

        usb::vendor->try_transmit();
        can::can1->try_transmit();
        usb::vendor->try_transmit();
        can::can2->try_transmit();
        usb::vendor->try_transmit();
        spi::spi1->update();
        usb::vendor->try_transmit();
        uart::uart1->try_transmit();
        usb::vendor->try_transmit();
        uart::uart2->try_transmit();
        usb::vendor->try_transmit();
        uart::uart_dbus->try_transmit();
    }
}

} // namespace librmcs::firmware
