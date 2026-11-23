# RMCS Board Ultra EtherCAT

The development transport supports one Ultra connected directly to a dedicated
Linux interface through ECAT0. The device runs the same librmcs Start/Keepalive
session and data protocol as USB. The EtherCAT app exposes only DFU Runtime over
USB; it does not switch transports at runtime. USB-only builds remain the
default on both Host and firmware. The current Ultra baseline does not provide
CAN, UART, GPIO or BMI088 data over EtherCAT.

## Hardware prerequisite

The firmware uses the HPM5E31 ESC with two JL1111 PHYs in MII mode. Clock the
ESC at 100 MHz and take its 25 MHz reference from `PA09` to both PHY XI inputs.
`PA23` must receive ECAT0 RX_ER. The two PHYs share reset (`PE00`), MDIO (`PA30`)
and MDC (`PA31`). Their active-high link outputs feed `PE01` and `PE04`; configure
PHY LED1 as link-only, not link/activity. The third ESC port is disabled.

**Do not flash the normal EtherCAT app onto a board with crossed clock/error
signals.** The prototype main board and PHY expansion KiCad projects mate J5
pad 20 to J2 pad 21 and J5 pad 21 to J2 pad 20. In those revisions, this routes
`PA23` to `ESC_REFCLK` and `PA09` to ECAT0 RX_ER. Verify or correct those two
connections before enabling the app. Driving REFCLK into a PHY output is not safe.

With both boards unpowered, verify continuity from `PA09` to expansion `J2.21`
(`ESC_REFCLK`) and from `PA23` to expansion `J2.20` (`ESC0_P0.MII_RX_ER`).
Confirm the two nets are isolated from each other before enabling the normal
app. The expansion cable connectors are `J3` for ECAT0 and `J4` for ECAT1;
check the harness pinout as well as the label on its network jack.

## Build and connect

Once the wiring is confirmed:

```sh
cmake --preset debug -S firmware/rmcs_board -B build/ultra-ecat-fw \
  -DBOARD=ultra -DLIBRMCS_ENABLE_ETHERCAT=ON -DLIBRMCS_ULTRA_ECAT_WIRING_CONFIRMED=ON
cmake --build build/ultra-ecat-fw --target rmcs_board_app

cmake --preset linux-debug -S host -B build/ultra-ecat-host -DLIBRMCS_ENABLE_ETHERCAT=ON
cmake --build build/ultra-ecat-host
```

Update only `rmcs_board_app.dfu` using the [Ultra DFU procedure](../firmware/rmcs_board/boards/ultra/README.md).
The same bootloader can load the USB-only app to return to USB data transport.

```cpp
#include <librmcs/board/rmcs_board_ultra.hpp>

librmcs::board::RmcsBoardUltra board{
    librmcs::board::EthercatOptions{.interface = "YOUR_DEDICATED_INTERFACE"}};
```

The interface must be up and must not carry normal network traffic or another
master. Check `ip -br link show <interface>` first. If it reports `NO-CARRIER`,
move the same known-good cable to ECAT0 (`J3`) and check again. A readable PHY
ID over MDIO does not establish that the cable link works. If only ECAT1 stays
down, check its `J4` harness and `L3` magnetics path before changing SOEM or PDO
settings. Test ECAT1 as an outgoing port to a downstream slave after the ECAT0
baseline is working; a direct master connection to ECAT1 is not validated.

The application needs permission to open a raw packet socket
(`CAP_NET_RAW`); SOEM also attempts to set interface flags, which requires
`CAP_NET_ADMIN`. With the tested USB Ethernet adapter, an already-up interface
worked with `CAP_NET_RAW` alone. Do not grant capabilities to a general-purpose
interpreter or run on a shared LAN. The Host verifies SII identity, fixed PDO
sizes and CoE software version (`0x100A`) before running in OP.

SOES and SOEM are pinned by commit and archive hash. Offline builds can supply
the corresponding extracted source via `FETCHCONTENT_SOURCE_DIR_LIBRMCS_SOES`
or `FETCHCONTENT_SOURCE_DIR_LIBRMCS_SOEM`. When linking the static Host SDK
manually, also link `librmcs_soem`, libusb, pthread, and rt. Review both upstream
licenses and their EtherCAT terms before distributing binaries.

## PDO map and session lifetime

SM0/1 are 128-byte CoE mailboxes at `0x1000`/`0x1080`. SM2 at `0x1100` maps
RxPDO `0x1600` (Host to device); SM3 at `0x1E00` maps TxPDO `0x1A00` (device to
Host). Each PDO is **1040 bytes** and has the same little-endian layout:

| Offset | Bytes | Field |
| --- | --- | --- |
| 0 | 1 | PDO version (1) |
| 1 | 1 | Reserved (0) |
| 2 | 2 | Payload length (0–1023) |
| 4 | 4 | Host-selected nonzero epoch |
| 8 | 4 | Sequence (0 for an idle frame) |
| 12 | 4 | Last accepted sequence from the peer (0 before the first) |
| 16 | 1023 | librmcs bytes; zero-filled after the payload |
| 1039 | 1 | Alignment padding (0) |

The PDO is fixed, does not interoperate with the EVK wire layout, and uses a
private engineering identity, not an allocated ETG vendor ID. The Host polls
nominally every 2 ms; no Distributed Clocks or hard real-time timing is offered.
The expected process-data working counter is 3 (outputs 2, inputs 1).

Each direction retains one in-flight payload until acknowledged and discards
duplicates. ACK means the peer accepted the PDO payload for protocol parsing;
it does not report that any peripheral accepted or completed a write. Buffers
are bounded: queue exhaustion is a transport failure/drop, not extra buffering
or a peripheral completion indication.

On missing process data the slave leaves OP after 100 ms and invalidates the
session. An unexpected OP loss or sustained bad PDO responses ends the Host
connection, and `connected()` becomes false. After restoring the link, destroy
and reconstruct the EtherCAT board object to start a fresh epoch and session.
Queued transfers are not replayed. ECAT1, daisy chaining, production PHY timing,
and physical peripheral transmission are outside the validated configuration.
