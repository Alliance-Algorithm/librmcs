# RMCS Board Ultra bootloader

Ultra uses the HPM5E31 built-in 1 MiB NOR. This board target currently builds
only the bootloader; the runtime app is not part of this target yet.

```sh
cmake --preset debug -S firmware/rmcs_board -B build/ultra -DBOARD=ultra
cmake --build build/ultra --target rmcs_board_bootloader
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

Hold SW4 (`USER_KEY`, PF26) at power-on to force DFU. With the board in normal
flash boot mode, `dfu-util -l` should show USB VID/PID `a11c:af02` and
`Internal Flash` (alt 0). In `auto` mode also verify a cold boot with no valid
app enters DFU without holding the key.
