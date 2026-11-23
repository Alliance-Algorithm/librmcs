# RMCS Board Ultra

Ultra uses the HPM5E31 built-in 1 MiB NOR. The runtime app exposes the existing
USB vendor interface and DFU Runtime interface. External peripherals are not
enabled in this image.

```sh
cmake --preset debug -S firmware/rmcs_board -B build/ultra -DBOARD=ultra
cmake --build build/ultra --target rmcs_board_bootloader rmcs_board_app
dfu-suffix -c build/ultra/app/output/rmcs_board_app.dfu
```

| Flash address | Purpose |
| --- | --- |
| `0x80000400` | NOR configuration, first file-backed bootloader load address |
| `0x80001000` | Boot ROM image header |
| `0x80003000` | Bootloader entry |
| `0x8001F000` through `0x8001FFFF` | Bootloader-managed app metadata |
| `0x80020000` through `0x800FFFFF` | App partition |

Use the ELF for initial debugger programming. The raw bootloader binary starts
at the first file-backed load address, not at `0x80000000`.

When upgrading from a build that used the `SYSCTL` mailbox, update the Ultra
bootloader and app together. The new pair uses `PGPR0` registers 12 and 13,
which retain the one-shot boot request across a system software reset.
Runtime detach does not work with mismatched mailbox versions.

Hold SW4 (`USER_KEY`, PF26) at power-on to force DFU. With the board in normal
flash boot mode, `dfu-util -l` should show USB VID/PID `a11c:af02` and
`Internal Flash` (alt 0). In `auto` mode also verify a cold boot with no valid
app enters DFU without holding the key.

After downloading `rmcs_board_app.dfu` to `Internal Flash` (alt 0), cold boot
without holding SW4. The runtime should enumerate as `a11c:af02`; check that
a matching-version Host can establish and keep a vendor session, and that DFU
Runtime detach returns the board to the bootloader. PF17 selects USB speed at
startup: low for high speed, high for full speed.

**USB acceptance (`auto`): passed on the Ultra prototype.** With no SW4 press
or active debugger session, `dfu-util -e` switched the app (480 Mb/s) to
`Found DFU` / `Internal Flash` (alt 0, 12 Mb/s). Downloading the app from that
DFU session returned the board to Runtime, and the matching-version Host SDK
established a vendor session and kept it active for 8 seconds with version
checks enabled. The app payload also matched Flash readback byte for byte.
Runtime at full speed, power-cycle boot, and `manual` mode were not tested in
this run.

Early prototype PCBs crossed D+ and D- at L2. Use a corrected PCB or a dedicated
crossed-data USB 2.0 test cable for those prototypes only.

## Status LED

The app uses the same status effects as Lite: green breathing during normal
operation, blinking yellow for an uplink buffer overflow, blinking cyan for
a downlink buffer overflow, yellow/cyan alternating for both, and red on an
assertion after LED initialization. Overflow indicators time out after five
seconds. The LED is not driven by the bootloader. Indicators apply to the
paths enabled in the current app; this change does not enable CAN or UART.

LED1 data uses PE05 (`PWM0.P5`) through R85. Counter 2 reloads generate PWM0
DMA2 requests, routed through TRGM0 DMA output 0 to HDMA. This sends 24 GRB bits
at 800 kHz without per-bit interrupts. The signal stays low between frames,
including when a delayed status tick makes two updates arrive close together.

**LED hardware acceptance: pending.** On a cold boot, check the green breathing
effect while the USB session is active. At LED1 DIN, verify 24 pulses with a
1.25 us bit period, roughly 0.4/0.8 us high pulses, and more than 280 us low
between frames. LED1 is powered by +5 V with a direct 3.3 V data signal, so
check the populated part's input high threshold if the waveform is correct but
the LED does not light.
