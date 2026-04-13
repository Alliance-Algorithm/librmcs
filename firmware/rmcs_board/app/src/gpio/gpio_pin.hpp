#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string_view>

#include <hpm_gpio_drv.h>
#include <hpm_gpio_regs.h>
#include <hpm_gpiom_drv.h>
#include <hpm_gpiom_soc_drv.h>
#include <hpm_ioc_regs.h>
#include <hpm_soc.h>

namespace librmcs::firmware {

class GpioPin {
public:
    static constexpr auto kPiocFunctionSocGpio = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(3);
    static constexpr auto kBiocFunctionSocGpio = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(3);
    static constexpr auto kIocFunctionGpio = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(0);

    constexpr GpioPin(
        gpiom_gpio_t controller, uintptr_t gpio_base, uint32_t pad, uint32_t port, uint32_t pin,
        bool active_high = true)
        : gpio_base_(gpio_base)
        , port_(port)
        , active_high_(active_high)
        , controller_(static_cast<uint32_t>(controller))
        , pad_(pad)
        , pin_(pin) {
        if (pad >= (1 << 16) || port >= (1 << 4) || pin >= (1 << 5)) [[unlikely]]
            __builtin_trap();
    }

    constexpr gpiom_gpio_t controller() const { return static_cast<gpiom_gpio_t>(controller_); }
    constexpr uintptr_t gpio_base() const { return gpio_base_; }
    constexpr uint32_t pad() const { return pad_; }
    constexpr uint32_t port() const { return port_; }
    constexpr uint32_t pin() const { return pin_; }
    constexpr bool active_high() const { return static_cast<bool>(active_high_); }

    GPIO_Type* gpio_instance() const { return reinterpret_cast<GPIO_Type*>(gpio_base()); }

    void configure_controller() const {
        gpiom_set_pin_controller(HPM_GPIOM, port(), pin(), controller());
    }

#ifdef HPM_PIOC
    void configure_pioc_function(uint32_t function = kPiocFunctionSocGpio) const {
        HPM_PIOC->PAD[pad()].FUNC_CTL = function;
    }
#endif

#ifdef HPM_BIOC
    void configure_bioc_function(uint32_t function = kBiocFunctionSocGpio) const {
        HPM_BIOC->PAD[pad()].FUNC_CTL = function;
    }
#endif

    void configure_ioc_function(uint32_t function = kIocFunctionGpio) const {
        HPM_IOC->PAD[pad()].FUNC_CTL = function;
    }

    void configure_pad_control(uint32_t control) const { HPM_IOC->PAD[pad()].PAD_CTL = control; }

    void configure_as_input() const { gpio_set_pin_input(gpio_instance(), port(), pin()); }
    void configure_as_output() const { gpio_set_pin_output(gpio_instance(), port(), pin()); }

    void configure_interrupt(gpio_interrupt_trigger_t trigger) const {
        gpio_config_pin_interrupt(gpio_instance(), port(), pin(), trigger);
    }

    bool read_pin() const {
        return static_cast<bool>(gpio_read_pin(gpio_instance(), port(), pin()));
    }

    void write_pin(bool high) const { gpio_write_pin(gpio_instance(), port(), pin(), high); }

    bool is_active() const { return read_pin() == active_high(); }

    void set_active(bool active) const { write_pin(active_high() ? active : !active); }

    void clear_interrupt_flag() const {
        gpio_clear_pin_interrupt_flag(gpio_instance(), port(), pin());
    }

    void enable_interrupt() const { gpio_enable_pin_interrupt(gpio_instance(), port(), pin()); }

    bool check_clear_interrupt_flag() const {
        return gpio_check_clear_interrupt_flag(gpio_instance(), port(), pin());
    }

private:
    uintptr_t gpio_base_;

    // Optimized for RV32I: 1-instruction extract for port / pin / active_high.
    uint32_t port_        : 4;
    uint32_t active_high_ : 1;
    uint32_t controller_  : 6;
    uint32_t pad_         : 16;
    uint32_t pin_         : 5;
};
static_assert(sizeof(GpioPin) == 8);

namespace internal {

constexpr uintptr_t gpio_base_from_controller(gpiom_gpio_t controller) {
    if (controller == gpiom_gpio_t::gpiom_soc_gpio0)
        return HPM_GPIO0_BASE;
    if (controller == gpiom_gpio_t::gpiom_core0_fast)
        return HPM_FGPIO_BASE;

#ifdef HPM_GPIO1_BASE
    if (controller == gpiom_gpio_t::gpiom_soc_gpio1)
        return HPM_GPIO1_BASE;
    if (controller == gpiom_gpio_t::gpiom_core1_fast)
        return HPM_FGPIO_BASE;
#endif

    return 0;
}

consteval bool is_ascii_space(char c) {
    return c == ' ' || c == '\t' || c == '\n' || c == '\r' || c == '\f' || c == '\v';
}

consteval bool has_outer_parentheses(std::string_view value) {
    if (value.size() < 2 || value.front() != '(' || value.back() != ')')
        return false;

    int depth = 0;
    for (size_t i = 0; i < value.size(); ++i) {
        if (value[i] == '(') {
            ++depth;
        } else if (value[i] == ')') {
            --depth;
            if (depth == 0 && i + 1 != value.size())
                return false;
            if (depth < 0)
                return false;
        }
    }

    return depth == 0;
}

consteval std::string_view trim_ascii_space(std::string_view value) {
    size_t begin = 0;
    size_t end = value.size();

    while (begin < end && is_ascii_space(value[begin]))
        ++begin;
    while (begin < end && is_ascii_space(value[end - 1]))
        --end;

    return value.substr(begin, end - begin);
}

consteval std::string_view trim_wrapping_parentheses(std::string_view value) {
    value = trim_ascii_space(value);

    while (has_outer_parentheses(value)) {
        value.remove_prefix(1);
        value.remove_suffix(1);
        value = trim_ascii_space(value);
    }

    return value;
}

consteval int digit_value(char c) {
    if (c >= '0' && c <= '9')
        return c - '0';
    if (c >= 'a' && c <= 'f')
        return c - 'a' + 10;
    if (c >= 'A' && c <= 'F')
        return c - 'A' + 10;
    return -1;
}

consteval bool is_integer_suffix(char c) {
    return c == 'u' || c == 'U' || c == 'l' || c == 'L' || c == 'z' || c == 'Z';
}

consteval uint32_t parse_pad_macro_value(
    std::string_view expanded, std::string_view macro_name, uint32_t fallback) {
    expanded = trim_wrapping_parentheses(expanded);
    if (expanded.empty() || expanded == macro_name)
        return fallback;

    size_t index = 0;
    int base = 10;
    if (expanded.size() >= 2 && expanded[0] == '0') {
        if (expanded[1] == 'x' || expanded[1] == 'X') {
            base = 16;
            index = 2;
        } else if (expanded[1] == 'b' || expanded[1] == 'B') {
            base = 2;
            index = 2;
        }
    }

    uint64_t result = 0;
    bool has_digit = false;
    for (; index < expanded.size(); ++index) {
        const char c = expanded[index];
        if (c == '\'')
            continue;

        const int digit = digit_value(c);
        if (digit < 0 || digit >= base)
            break;

        has_digit = true;
        if (result > (static_cast<uint64_t>(std::numeric_limits<uint32_t>::max()) - digit)
                         / static_cast<uint64_t>(base)) {
            return fallback;
        }
        result = (result * static_cast<uint64_t>(base)) + static_cast<uint64_t>(digit);
    }

    if (!has_digit)
        return fallback;

    for (; index < expanded.size(); ++index) {
        const char c = expanded[index];
        if (c == '\'' || is_ascii_space(c))
            continue;
        if (!is_integer_suffix(c))
            return fallback;
    }

    return static_cast<uint32_t>(result);
}

template <char port_name>
struct GpioPortTraits {
    static constexpr bool has_pad(uint32_t) { return false; }
};

#define LIBRMCS_INTERNAL_STR_DETAIL(x) #x
#define LIBRMCS_INTERNAL_STR(x)        LIBRMCS_INTERNAL_STR_DETAIL(x)

#define LIBRMCS_INTERNAL_PORT_CHAR_DETAIL(letter) #letter
#define LIBRMCS_INTERNAL_PORT_CHAR(letter)        LIBRMCS_INTERNAL_PORT_CHAR_DETAIL(letter)[0]

#define LIBRMCS_INTERNAL_GET_PAD_VAL(macro_name)          \
    ::librmcs::firmware::internal::parse_pad_macro_value( \
        LIBRMCS_INTERNAL_STR(macro_name), #macro_name, kInvalidPad)

#define LIBRMCS_INTERNAL_PAD_ENTRY(letter, pin) LIBRMCS_INTERNAL_GET_PAD_VAL(IOC_PAD_P##letter##pin)

#define LIBRMCS_INTERNAL_DEFINE_GPIO_PORT_TRAITS(letter)                                     \
    template <>                                                                              \
    struct GpioPortTraits<LIBRMCS_INTERNAL_PORT_CHAR(letter)> {                              \
        static_assert(GPIO_DI_GPIO##letter == GPIO_DO_GPIO##letter);                         \
        static_assert(GPIO_DI_GPIO##letter == GPIO_OE_GPIO##letter);                         \
        static_assert(GPIO_DI_GPIO##letter == GPIO_IF_GPIO##letter);                         \
        static_assert(GPIO_DI_GPIO##letter == GPIO_IE_GPIO##letter);                         \
        static_assert(GPIO_DI_GPIO##letter == GPIO_PL_GPIO##letter);                         \
        static_assert(GPIO_DI_GPIO##letter == GPIO_TP_GPIO##letter);                         \
        static_assert(GPIO_DI_GPIO##letter == GPIO_AS_GPIO##letter);                         \
        static_assert(GPIO_DI_GPIO##letter == GPIO_PD_GPIO##letter);                         \
        static_assert(GPIO_DI_GPIO##letter == GPIOM_ASSIGN_GPIO##letter);                    \
        static_assert(GPIO_DI_GPIO##letter < 256);                                           \
        static constexpr uint32_t kInvalidPad = std::numeric_limits<uint32_t>::max();        \
        static constexpr std::array<uint32_t, 32> kPadByPin = {                              \
            LIBRMCS_INTERNAL_PAD_ENTRY(letter, 00), LIBRMCS_INTERNAL_PAD_ENTRY(letter, 01),  \
            LIBRMCS_INTERNAL_PAD_ENTRY(letter, 02), LIBRMCS_INTERNAL_PAD_ENTRY(letter, 03),  \
            LIBRMCS_INTERNAL_PAD_ENTRY(letter, 04), LIBRMCS_INTERNAL_PAD_ENTRY(letter, 05),  \
            LIBRMCS_INTERNAL_PAD_ENTRY(letter, 06), LIBRMCS_INTERNAL_PAD_ENTRY(letter, 07),  \
            LIBRMCS_INTERNAL_PAD_ENTRY(letter, 08), LIBRMCS_INTERNAL_PAD_ENTRY(letter, 09),  \
            LIBRMCS_INTERNAL_PAD_ENTRY(letter, 10), LIBRMCS_INTERNAL_PAD_ENTRY(letter, 11),  \
            LIBRMCS_INTERNAL_PAD_ENTRY(letter, 12), LIBRMCS_INTERNAL_PAD_ENTRY(letter, 13),  \
            LIBRMCS_INTERNAL_PAD_ENTRY(letter, 14), LIBRMCS_INTERNAL_PAD_ENTRY(letter, 15),  \
            LIBRMCS_INTERNAL_PAD_ENTRY(letter, 16), LIBRMCS_INTERNAL_PAD_ENTRY(letter, 17),  \
            LIBRMCS_INTERNAL_PAD_ENTRY(letter, 18), LIBRMCS_INTERNAL_PAD_ENTRY(letter, 19),  \
            LIBRMCS_INTERNAL_PAD_ENTRY(letter, 20), LIBRMCS_INTERNAL_PAD_ENTRY(letter, 21),  \
            LIBRMCS_INTERNAL_PAD_ENTRY(letter, 22), LIBRMCS_INTERNAL_PAD_ENTRY(letter, 23),  \
            LIBRMCS_INTERNAL_PAD_ENTRY(letter, 24), LIBRMCS_INTERNAL_PAD_ENTRY(letter, 25),  \
            LIBRMCS_INTERNAL_PAD_ENTRY(letter, 26), LIBRMCS_INTERNAL_PAD_ENTRY(letter, 27),  \
            LIBRMCS_INTERNAL_PAD_ENTRY(letter, 28), LIBRMCS_INTERNAL_PAD_ENTRY(letter, 29),  \
            LIBRMCS_INTERNAL_PAD_ENTRY(letter, 30), LIBRMCS_INTERNAL_PAD_ENTRY(letter, 31)}; \
        static constexpr bool has_pad(uint32_t pin) {                                        \
            return pin < kPadByPin.size() && kPadByPin[pin] != kInvalidPad;                  \
        }                                                                                    \
        static constexpr uint32_t pad(uint32_t pin) { return kPadByPin[pin]; }               \
        static constexpr uint32_t kPort = GPIO_DI_GPIO##letter;                              \
    };

#ifdef GPIO_DI_GPIOA
LIBRMCS_INTERNAL_DEFINE_GPIO_PORT_TRAITS(A)
#endif

#ifdef GPIO_DI_GPIOB
LIBRMCS_INTERNAL_DEFINE_GPIO_PORT_TRAITS(B)
#endif

#ifdef GPIO_DI_GPIOC
LIBRMCS_INTERNAL_DEFINE_GPIO_PORT_TRAITS(C)
#endif

#ifdef GPIO_DI_GPIOD
LIBRMCS_INTERNAL_DEFINE_GPIO_PORT_TRAITS(D)
#endif

#ifdef GPIO_DI_GPIOE
LIBRMCS_INTERNAL_DEFINE_GPIO_PORT_TRAITS(E)
#endif

#ifdef GPIO_DI_GPIOF
LIBRMCS_INTERNAL_DEFINE_GPIO_PORT_TRAITS(F)
#endif

#ifdef GPIO_DI_GPIOG
LIBRMCS_INTERNAL_DEFINE_GPIO_PORT_TRAITS(G)
#endif

#ifdef GPIO_DI_GPIOH
LIBRMCS_INTERNAL_DEFINE_GPIO_PORT_TRAITS(H)
#endif

#ifdef GPIO_DI_GPIOI
LIBRMCS_INTERNAL_DEFINE_GPIO_PORT_TRAITS(I)
#endif

#ifdef GPIO_DI_GPIOJ
LIBRMCS_INTERNAL_DEFINE_GPIO_PORT_TRAITS(J)
#endif

#ifdef GPIO_DI_GPIOK
LIBRMCS_INTERNAL_DEFINE_GPIO_PORT_TRAITS(K)
#endif

#ifdef GPIO_DI_GPIOL
LIBRMCS_INTERNAL_DEFINE_GPIO_PORT_TRAITS(L)
#endif

#ifdef GPIO_DI_GPIOM
LIBRMCS_INTERNAL_DEFINE_GPIO_PORT_TRAITS(M)
#endif

#ifdef GPIO_DI_GPION
LIBRMCS_INTERNAL_DEFINE_GPIO_PORT_TRAITS(N)
#endif

#ifdef GPIO_DI_GPIOO
LIBRMCS_INTERNAL_DEFINE_GPIO_PORT_TRAITS(O)
#endif

#ifdef GPIO_DI_GPIOP
LIBRMCS_INTERNAL_DEFINE_GPIO_PORT_TRAITS(P)
#endif

#ifdef GPIO_DI_GPIOQ
LIBRMCS_INTERNAL_DEFINE_GPIO_PORT_TRAITS(Q)
#endif

#ifdef GPIO_DI_GPIOR
LIBRMCS_INTERNAL_DEFINE_GPIO_PORT_TRAITS(R)
#endif

#ifdef GPIO_DI_GPIOS
LIBRMCS_INTERNAL_DEFINE_GPIO_PORT_TRAITS(S)
#endif

#ifdef GPIO_DI_GPIOT
LIBRMCS_INTERNAL_DEFINE_GPIO_PORT_TRAITS(T)
#endif

#ifdef GPIO_DI_GPIOU
LIBRMCS_INTERNAL_DEFINE_GPIO_PORT_TRAITS(U)
#endif

#ifdef GPIO_DI_GPIOV
LIBRMCS_INTERNAL_DEFINE_GPIO_PORT_TRAITS(V)
#endif

#ifdef GPIO_DI_GPIOW
LIBRMCS_INTERNAL_DEFINE_GPIO_PORT_TRAITS(W)
#endif

#ifdef GPIO_DI_GPIOX
LIBRMCS_INTERNAL_DEFINE_GPIO_PORT_TRAITS(X)
#endif

#ifdef GPIO_DI_GPIOY
LIBRMCS_INTERNAL_DEFINE_GPIO_PORT_TRAITS(Y)
#endif

#ifdef GPIO_DI_GPIOZ
LIBRMCS_INTERNAL_DEFINE_GPIO_PORT_TRAITS(Z)
#endif

#undef LIBRMCS_INTERNAL_DEFINE_GPIO_PORT_TRAITS

#undef LIBRMCS_INTERNAL_PAD_ENTRY
#undef LIBRMCS_INTERNAL_GET_PAD_VAL

#undef LIBRMCS_INTERNAL_PORT_CHAR
#undef LIBRMCS_INTERNAL_PORT_CHAR_DETAIL

#undef LIBRMCS_INTERNAL_STR
#undef LIBRMCS_INTERNAL_STR_DETAIL

} // namespace internal

template <gpiom_gpio_t controller, char port_name, uint32_t pin, bool active_high = true>
requires(
    internal::gpio_base_from_controller(controller) != 0
    && internal::GpioPortTraits<port_name>::has_pad(pin))
consteval GpioPin make_gpio_pin() {
    using PortTraits = internal::GpioPortTraits<port_name>;

    return GpioPin{
        controller,
        internal::gpio_base_from_controller(controller),
        PortTraits::pad(pin),
        PortTraits::kPort,
        pin,
        active_high,
    };
}

} // namespace librmcs::firmware
