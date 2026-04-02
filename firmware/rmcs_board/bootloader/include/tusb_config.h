#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#ifndef CFG_TUSB_MCU
# error CFG_TUSB_MCU must be defined
#endif

#ifndef BOARD_DEVICE_RHPORT_NUM
# define BOARD_DEVICE_RHPORT_NUM 0
#endif

#ifndef BOARD_DEVICE_RHPORT_SPEED
# define BOARD_DEVICE_RHPORT_SPEED OPT_MODE_FULL_SPEED
#endif

#if BOARD_DEVICE_RHPORT_NUM == 0
# define CFG_TUSB_RHPORT0_MODE (OPT_MODE_DEVICE | BOARD_DEVICE_RHPORT_SPEED)
#elif BOARD_DEVICE_RHPORT_NUM == 1
# define CFG_TUSB_RHPORT1_MODE (OPT_MODE_DEVICE | BOARD_DEVICE_RHPORT_SPEED)
#else
# error "Incorrect RHPort configuration"
#endif

#define CFG_TUSB_OS OPT_OS_NONE

#ifndef CFG_TUSB_MEM_SECTION
# define CFG_TUSB_MEM_SECTION __attribute__((section(".noncacheable.non_init")))
#endif

#ifndef CFG_TUSB_MEM_ALIGN
# define CFG_TUSB_MEM_ALIGN __attribute__((aligned(4)))
#endif

#ifndef CFG_TUD_ENDPOINT0_SIZE
# define CFG_TUD_ENDPOINT0_SIZE 64
#endif

#define CFG_TUD_CDC         0
#define CFG_TUD_MSC         0
#define CFG_TUD_HID         0
#define CFG_TUD_MIDI        0
#define CFG_TUD_VENDOR      0
#define CFG_TUD_DFU_RUNTIME 0
#define CFG_TUD_DFU         1

#define CFG_TUD_DFU_XFER_BUFSIZE 1024

#ifdef __cplusplus
}
#endif
