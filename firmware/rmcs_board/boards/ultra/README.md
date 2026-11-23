# RMCS Board Ultra

Ultra uses the HPM5E31 built-in 1 MiB NOR. The runtime app exposes the existing
USB vendor interface and DFU Runtime interface. External peripherals and the
status LED are not enabled in this image.

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

## UART and DBUS

The port names below follow the RMCS physical numbering convention.

| Port | Pin order |
| --- | --- |
| **UART0** | 1 RX, 2 TX, 3 GND, 4 +5 V |
| **UART1** | 1 GND, 2 TX, 3 RX |
| **UART2** | 1 RX, 2 TX, 3 GND, 4 +5 V |
| **DBUS** | 1 GND, 2 +5 V, 3 RX |

TX/RX jumpers were verified electrically on the three UART ports in separate
tests. Ordinary UART data transfer has not been tested yet.

With UART1 pin 2 connected to DBUS pin 3, the DBUS hardware receive path
passed 40/40 test bytes at 100000 baud, eight data bits, even parity and one
stop bit, with no parity, framing or overrun errors. The test generated
inverted serial waveforms from the UART1 TX pin using a hardware timer to
account for the onboard DBUS inverter. It did not test the UART1 transmitter,
the production DMA receive path, USB data upload or remote-control decoding.
No Flash was written; the original peripheral settings were restored.

The board header declares the port mappings. The USB-only app above still
does not initialize or expose UART or DBUS.
