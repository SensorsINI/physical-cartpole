# Firmware

CartPoleFirmware for STM32 and Zynq, build scripts, and Vitis platforms.
Flashing and FPGA rebuild: [Docs/firmware-and-flash.md](../Docs/firmware-and-flash.md).

## Chip selection

Edit [Src/CartPoleFirmware/hardware_bridge.h](Src/CartPoleFirmware/hardware_bridge.h):

| Target | Defines |
|---|---|
| Zybo Z7-20 (Development) | `#define ZYNQ`, `#define ZYBO_Z720`, `#define USE_EXTERNAL_INTERFACE` |
| STM32 | `#define STM` (comment out `ZYNQ`) |

Match [Driver/globals.py](../Driver/globals.py) `CHIP` and `ZYNQ_BOARD`.

Plant defaults (hanging, motor map, encoder range):
[Src/CartPoleFirmware/parameters.c](Src/CartPoleFirmware/parameters.c).

## Show image (AMP production)

Typical Development flow:

```bash
# Build CPU0 + CPU1 production ELFs (CPU1 blob linked into CPU0)
RPGD_AMP_PRODUCTION=1 Scripts/build_rpgd_amp_elfs.sh

# JTAG load into RAM (no flash write)
Scripts/program_rpgd_amp_production.sh

# Or flash QSPI BOOT.BIN (JP5 = JTAG while programming)
Scripts/program_show_qspi.sh
```

Output ELF: `CartPoleFirmware_rpgd_amp_cpu0.elf`. BIF template:
[Scripts/cartpole_qspi.bif](Scripts/cartpole_qspi.bif).

Platform: `VitisProjects/cartpole_zybo_secloc2026`. Prebuilt bitstream:
[../FPGA/bitstreams/cartpole_short_pole_secloc.bit](../FPGA/bitstreams/cartpole_short_pole_secloc.bit).

## Script index

| Script | Purpose |
|---|---|
| `build_rpgd_amp_elfs.sh` | Build AMP CPU0/CPU1 ELFs; set `RPGD_AMP_PRODUCTION=1` for show link |
| `build_rpgd_amp_production.sh` | Wrapper for production AMP build |
| `program_rpgd_amp_production.sh` | JTAG load show image to RAM |
| `program_show_qspi.sh` | Flash `BOOT.BIN` to QSPI |
| `program_qspi_boot.tcl` | Vivado hw_server flash helper |
| `setup_rpgd_amp_platform.tcl` | xsct: configure CPU1 domain on platform |
| `build_rpgd_on_target_elf.sh` | On-target RPGD test ELF |
| `launch_rpgd_amp_jtag.tcl` | JTAG launch helper |
| `generate_rpgd_amp_bsp.tcl` | BSP generation |
| `run_rpgd_amp_stage_a.sh` | Staged AMP build step |

## On-chip controllers (source)

| Controller | Primary files |
|---|---|
| Show mux / profiles | [Src/CartPoleFirmware/controller_profiles.c](Src/CartPoleFirmware/controller_profiles.c) |
| Motor loop / UART | [Src/CartPoleFirmware/control.c](Src/CartPoleFirmware/control.c) |
| Dense-8 C (SW1) | [Src/General/neural_controller_C.c](Src/General/neural_controller_C.c), [NC_C/](Src/General/NC_C/) |
| LSTM C (SW2) | [Src/General/neural_controller_LSTM_C.c](Src/General/neural_controller_LSTM_C.c), [NC_LSTM/](Src/General/NC_LSTM/) |
| PL imitator (SW3) | [Src/Zynq/neural_imitator.c](Src/Zynq/neural_imitator.c) |
| RPGD (SW0, CPU1) | Linked worker in AMP build |

## STM32

CubeIDE project: [CubeIDE/CartPoleFirmware](CubeIDE/CartPoleFirmware). See
[firmware-and-flash.md](../Docs/firmware-and-flash.md#stm32).

## Timing test

Enable `#define TIMING_TEST` in `hardware_bridge.h` and follow
[Src/Embedded_Controller/TIMING_TEST_README.md](Src/Embedded_Controller/TIMING_TEST_README.md)
(motor stays off).
