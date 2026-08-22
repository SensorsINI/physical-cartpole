# SecLoc three-block HLS chain

The monolithic `secloc_frontend` IP was split into three stream-connected blocks.
Sources live under `FPGA/CustomIPs/{secloc_shell_hls,secloc_gate_hls,nn_marshal_hls}/`.

## Data path

```
PS (AXI-Lite) → secloc_shell → secloc_gate → nn_marshal → controller_axis → hls4ml VHDL
```

Shared packet layout: `secloc_stream_protocol.h` (`SECLOC_STREAM_VERSION = 2`).

## Build all three IPs

```bash
FPGA/CustomIPs/build_secloc_chain.sh
```

Only `nn_marshal` must be re-synthesized after a network swap (when
`nn_marshal_config.h` changes). Shell and gate are network-agnostic.

## C simulation (system g++ — Vitis 2020.1 csim is too old for this glibc)

```bash
FPGA/CustomIPs/secloc_shell_hls/run_tb.sh
```

## Network swap + bitstream

1. Run hls4ml conversion (auto-generates `nn_marshal_config.h` in the network folder).
2. Ensure the new VHDL sources are in the Vivado project fileset.
3. Rebuild marshal + bitstream:

```bash
FPGA/VivadoProjects/swap_nn_and_build.sh [FPGA/NeuralNetworks/<network>]
```

Override the network folder at HLS time with `SECLOC_NN_NETWORK_DIR`.

## Vivado block design

`FPGA/VivadoProjects/CartpoleDriverZynq_AXIS_Zedboard.tcl` instantiates
`SECLOC_SHELL_0`, `SECLOC_GATE_0`, `NN_MARSHAL_0` at the former frontend address
(`0x40410000` on shell only). After BD TCL changes, recreate the project or run:

```bash
cd FPGA/VivadoProjects
vivado -mode batch -source refresh_ip_and_build.tcl
```

## Firmware

`Firmware/Src/Zynq/secloc_frontend_link.c` talks to the shell register map
(`xsecloc_shell_hw.h`). `hw_platform_config.h` detects `SECLOC_SHELL_0` with
fallback to the retired `SECLOC_FRONTEND_0` symbol.
