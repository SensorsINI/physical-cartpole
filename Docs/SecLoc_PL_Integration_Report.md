# SecLoc in Zynq PL — Three-Block HLS Chain + VHDL Network

**Decision path:** separate-AXI-block → adapter wrapper with in-band
parameters → hardware normalization → fused frontend → **three stream-connected
HLS blocks** (shell / gate / marshal) with the hls4ml network unchanged in VHDL.

Three HLS IPs replace the retired monolithic `secloc_frontend`:

| Block | Role | Re-synth on network swap? |
|---|---|---|
| `secloc_shell` | AXI-Lite register map ↔ 14-word request / 4-word response packets | No |
| `secloc_gate` | SecLoc gate (`secloc_logic.c`) + zero-order hold + watchdog | No |
| `nn_marshal` | Per-network normalize / quantize / denormalize | **Yes** |

```
PS (Cortex-A9)
  │ AXI-Lite (same register map as the retired frontend)
  ▼
secloc_shell (HLS, ap_ctrl_hs)
  │ AXIS: 14-word request / 4-word response (secloc_stream_protocol.h)
  ▼
secloc_gate (HLS, ap_ctrl_none — network-agnostic)
  │ AXIS downstream: 7 raw float32 beats (TLAST)
  ▼
nn_marshal (HLS, ap_ctrl_none — per-network)
  │ AXIS: 7 × ap_fixed codes in 32-bit beats  ▲  1 × output code
  ▼
controller_axis.vhd (EXISTING, unchanged)
  ▼
controller_adapter_cp → myproject.vhd (hls4ml VHDL, unchanged)
```

**Latency overhead:** ~30–60 extra PL cycles for the two additional stream hops
(II=1 serialization + handshake), negligible vs ~100-cycle inference and the
5 ms control loop. PS AXI-Lite traffic is unchanged.

### Per-network fixed-point format

Each network folder ships `nn_marshal_config.h` (auto-generated on hls4ml conversion):

```c
#define NN_IO_INPUT_COUNT        7
#define NN_IO_OUTPUT_COUNT       1
#define NN_IO_INPUT_TOTAL_BITS   12   /* ap_fixed<12,2> */
#define NN_IO_INPUT_INT_BITS     2
#define NN_IO_OUTPUT_TOTAL_BITS  12
#define NN_IO_OUTPUT_INT_BITS    2
static const float NN_NORM_A[7] = { ... };
static const float NN_NORM_B[7] = { ... };
```

`nn_marshal` includes these at HLS compile time via `nn_fixed_format.h`.
Input and output widths may differ. `controller_io_parameters.vhd` must match.
On swap: re-run hls4ml conversion (which regenerates the header), then
`nn_marshal_hls/run_hls.tcl`, then rebuild.

### Stream packet layout (`secloc_stream_protocol.h`)

**Shell → gate request (14 words, TLAST on last):**

| Word | Field |
|---|---|
| 0 | `control_flags` |
| 1 | `tick` |
| 2–5 | `log_base`, `ref_period_ticks`, `dead_ang`, `dead_pos` |
| 6 | `angle` |
| 7–13 | `angleD`, `angle_cos`, `angle_sin`, `position`, `positionD`, `te`, `tp` |

**Gate → shell response (4 words):** `Q`, `status`, `update_count`, `nn_wait_cycles`

**Gate ↔ marshal:** 7 raw float32 in, 1 float32 Q out (physical units).

---

## 1. Why this shape

### 1.1 HLS for the SecLoc/interface part (vs hand VHDL)

- **Bit-exact gate parity for free.** The HLS top `#include`s the firmware's
  `secloc_logic.c` unmodified. Vitis HLS float32 add/mul/div/compare are
  IEEE-754 compliant — the same arithmetic the ARM FPU does — so PL and C
  gate decisions are bit-identical by construction. The entire fixed-point
  apparatus of the earlier VHDL plan (Q20 formats, milli-scaled ratio test,
  quantized reference model, near-threshold disagreement analysis) is
  deleted, along with its verification burden. The float divisions in the
  ratio test cost ~15–30 cycles each — irrelevant at a 5 ms loop.
- **Normalizer parity likewise**: `a[i]·x + b[i]` in float + round to
  `ap_fixed<12,2>` is literally today's `neural_imitator.c` marshalling,
  now in PL, plus saturation (which the current PS code lacks).
- **The AXI-Lite interface is generated, not hand-written**: `s_axilite`
  pragmas produce the register map, slave logic, and C driver headers. The
  planned hand-extension of `controller_axilite` (FSM, auto-increment
  side-channels, shell testbench) disappears.
- **Established flow**: the repo already builds Vitis HLS 2020.1 IPs
  (motor, encoder, median filter) with C testbenches.

### 1.2 Keeping the network in VHDL (vs a fused all-HLS design)

A fused HLS design (frontend calling `myproject()` as C++) was considered and
rejected: the network folders ship only generated VHDL (no hls4ml C++
`firmware/` sources are committed), and fusing would change the network-swap
flow from "swap VHDL files + rebuild" to "regenerate C++ + re-run HLS of the
combined project". Keeping the boundary at RTL level preserves the current
swap discipline (`swap_nn_and_build.tcl`) and leaves the deployed, validated
network bit-for-bit identical.

The boundary costs almost nothing because **`controller_axis.vhd` already
exists**: it accepts one input code per 32-bit AXIS beat (TLAST-terminated),
drives the `controller_adapter_cp` → `myproject` ap_ctrl_hs handshake, and
streams the result back. It is network-agnostic via
`controller_io_parameters.vhd`. Frontend and network share FCLK0, so the
streams connect directly in the block design (an AXIS register slice is
optional insurance).

Driving `myproject`'s ap_ctrl_hs pins directly from HLS port-level protocols
would avoid the bridge but is the fiddly corner of HLS (protocol regions,
hand-managed handshake waits) — the existing bridge is the robust route.

---

## 2. The three HLS IPs

### 2.1 `secloc_shell` — PS register front-end

Top function: `secloc_shell` (`FPGA/CustomIPs/secloc_shell_hls/secloc_shell.cpp`).

- **Interface**: `s_axilite` bundle `ctrl` with `ap_ctrl_hs` (same scalar signature
  as the retired monolithic frontend); AXIS `shell_req` / `shell_resp`.
- **Behaviour**: on `ap_start`, pack the 9 state floats + tick + control flags +
  four gate parameters into a 14-word request packet
  (`secloc_stream_protocol.h`), push downstream, block until the 4-word response
  returns, then latch `Q`, `status`, `update_count`, `nn_wait_cycles` into the
  output registers.
- **Re-synth on network swap**: no.

### 2.2 `secloc_gate` — gate + zero-order hold + watchdog

Top function: `secloc_gate` (`FPGA/CustomIPs/secloc_gate_hls/secloc_gate.cpp`).

- **Interface**: AXIS in/out only (`ap_ctrl_none` — always ready between shell
  and marshal).
- **Gate source**: `#include "secloc_logic.c"` from `Firmware/Src/General/`
  (single source of truth). `gate_should_sample` is `INLINE off` so csynth
  reports isolated gate latency.
- **On fire**: forward 7 raw float32 NN inputs downstream (TLAST on last beat),
  wait for one float32 `Q` beat from marshal, count `nn_wait_cycles` while
  polling the marshal→network path (4096-cycle watchdog → `SECLOC_STATUS_NN_TIMEOUT`).
- **On skip**: return the held `Q` immediately; status flags mirror firmware
  telemetry (`skipped_update`, `gate_skipped`).
- **Degenerate always-fire mode** (`log_base = 1.0`, `dead_* = 0`,
  `ref_period_ticks = 0`): behaves as a plain neural imitator.
- **Re-synth on network swap**: no.

### 2.3 `nn_marshal` — per-network normalize / quantize / denormalize

Top function: `nn_marshal` (`FPGA/CustomIPs/nn_marshal_hls/nn_marshal.cpp`).

- **Interface**: AXIS `gate_req` / `gate_resp` (float32 physical units) and
  `nn_req` / `nn_resp` (quantized codes in 32-bit beats, matching
  `controller_axis.vhd`).
- **Constants**: `#include "nn_marshal_config.h"` from the deployed network
  folder (auto-generated). Widths via `nn_fixed_format.h`
  (`NN_IO_INPUT_TOTAL_BITS`, `NN_IO_OUTPUT_TOTAL_BITS`, etc.).
- **On transaction**: read 7 floats from gate, normalize+quantize, push to NN,
  read one output code, denormalize, return float `Q` to gate.
- **Re-synth on network swap**: **yes** — run `nn_marshal_hls/run_hls.tcl`
  (or `swap_nn_and_build.sh`).

### 2.4 Register map / wire cost

Vitis generates the shell map (offsets in `xsecloc_shell_hw.h`, mirrored in
`secloc_frontend_link.c`). Per control step the PS writes 9 state words +
CTRL start, polls done, reads 4 outputs — ~15 AXI-Lite accesses ≈ 15–30 µs at
FCLK0 = 10 MHz (≪ 5 ms loop). Gate parameters are sticky registers written on
config change only. Only the shell has AXI-Lite; gate and marshal are pure
stream blocks.

### 2.5 Normalization coefficients

Baked as **HLS compile-time defaults from `nn_marshal_config.h`** in the network
folder, emitted by `FPGA/scripts/generate_nn_marshal_config.py` at the end of
hls4ml conversion. Rationale: per-network calibration, not runtime tuning.
A network swap re-runs csynth of `nn_marshal` only (minutes; Vivado rebuild
dominates). `swap_nn_and_build.sh` runs marshal HLS then upgrades the IP in
the block design. The script fails loudly if the header is missing.

---

## 3. Firmware changes

1. **`hw_platform_config.h`** — map
   `XPAR_..._SECLOC_SHELL_0_S_AXI_CTRL_BASEADDR` (fallback:
   `SECLOC_FRONTEND_0` for legacy bitstreams) →
   `HW_SECLOC_FRONTEND_BASEADDR` / `HW_HAS_SECLOC_FRONTEND`. The shell
   exposes a version field in `status` [31:24] for runtime sanity check
   (`SFE_EXPECTED_VERSION = 2`).
2. **New `Firmware/Src/Zynq/secloc_frontend_link.c/.h`** — thin MMIO driver
   using the generated offsets: write state floats (as bit patterns), start,
   poll CTRL ap_done, read `Q/status/update_count/nn_wait_cycles`; plus
   `SetParams(...)` (on config change), `ResetGate()`. Tick computed with a
   shared helper mirroring `secloc_tick_from_time()` exactly.
3. **`secloc_controller.c` / `secloc_controller_pl.c`** — the SW core and
   the PL backend module are separate files talking through
   `secloc_controller_internal.h`; the backend switch (default in
   `secloc_defaults.h`, runtime-switchable like the inner controller)
   selects: `SECLOC_BACKEND_SW` (gate + inner controller on the CPU — STM
   and PC-ctypes builds unaffected, the PL module is inert there),
   `SECLOC_BACKEND_PL` (delegate the step to the shell + PL chain),
   `SECLOC_BACKEND_PL_SHADOW` (control from the PL, SW gate stepped in
   parallel and per-step agreement counted; with the bit-exact float gate
   the expectation is **zero** mismatches, which makes shadow mode a sharp
   go/no-go check rather than a statistics exercise). The selection is a
   request with **no SW fallback**: if a PL backend is selected while the
   PL block is absent or a transaction fails, the step outputs zero force
   and is flagged (telemetry bit 3, `secloc_pl_fault_count`,
   `CMD_GET_SECLOC_INFO`) — a dead cart is obvious even standalone, and
   the wrong implementation can never run unnoticed.
4. **`cmd_SetSeclocConfig`** — unchanged wire format; handler additionally
   forwards the four parameters to the frontend registers when present.
5. **`neural_imitator.c`** — the HLS4ML branch delegates to the shell in
   always-fire mode when it is the only NN interface; the
   `hls_normalize_a/b` / `hls_denormalize_A/B` constant blocks and the
   float→fixed marshalling on the PS are deleted (now in `nn_marshal`).
6. `hw_accel_link.c` keeps supporting the old shells for other accelerators
   (EdgeDRNN, DiffLogic paths untouched).

---

## 4. Block design / build changes

1. Three HLS IPs under `FPGA/CustomIPs/`:
   `secloc_shell_hls/`, `secloc_gate_hls/`, `nn_marshal_hls/` (each with
   `.cpp`, `.h`, `_tb.cpp`, `run_hls.tcl`). Build all:
   `FPGA/CustomIPs/build_secloc_chain.sh`.
2. In `HARDWARE_ACCEL`: **replace** the retired `SECLOC_FRONTEND_0` with
   `SECLOC_SHELL_0` + `SECLOC_GATE_0` + `NN_MARSHAL_0` +
   `CONTROLLER_AXIS_0` (module reference). Stream wiring:
   shell↔gate↔marshal↔`controller_axis`. AXI-Lite only to shell at
   `0x40410000`.
3. Rebuild bitstream (`rebuild_secloc_three_block.tcl` after HLS export),
   export XSA, regenerate BSP → `XPAR_HARDWARE_ACCEL_SECLOC_SHELL_0_*`.
4. Network swap flow: hls4ml conversion regenerates VHDL +
   `nn_marshal_config.h`; then `swap_nn_and_build.sh` re-runs marshal csynth
   and Vivado. Shell and gate IPs are unchanged.

`controller_axilite.vhd` remains in the tree for non-SecLoc configurations
but is no longer instantiated.

---

## 5. Verification

1. **C lockstep TB** — `secloc_shell_hls/run_tb.sh` (system g++): per-block
   tests for shell and marshal, plus `secloc_three_block_tb.cpp` chaining
   shell → gate → marshal with a mock NN stream. Gate reference is a second
   `secloc_logic.c` instance; 20k-step randomized lockstep, all green.
2. **HLS csynth** — all three IPs exported. Latency (100 MHz synth clock):
   shell ~29 cycles; gate handle 28 cycles (skip) to ~4221 (fire + NN
   watchdog timeout); marshal compute path ~30 cycles plus network wait.
   Gate `INLINE off` gives an isolated latency line in the gate report.
3. **Bridge-level sim** — `controller_axis` + `controller_adapter_cp` +
   `myproject` unchanged and hardware-proven; optional integration sim with
   marshal RTL + bridge + mock network.
4. **On-rig shadow campaign** — `SECLOC_BACKEND_PL_SHADOW`: PL chain and SW
   gate stepped in parallel; bit-exact float arithmetic → zero expected
   mismatches. Promote to `SECLOC_BACKEND_PL` after a clean run;
   `nn_wait_cycles` and skip rates recorded.

The C↔Python parity suite, the Python stack, and the network VHDL stay
untouched throughout.

---

## 6. Phasing

| Phase | Deliverable | Risk |
|---|---|---|
| 1 | Tick-entry refactor of `secloc_logic.c` (shared by firmware + HLS); three-block C TB green | Done |
| 2 | `nn_marshal_config.h` generation + marshal csim vs PS float path; `swap_nn_and_build.sh` | Done |
| 3 | csynth + IP export for shell/gate/marshal; latency report | Done |
| 4 | Integration TB: shell + gate + marshal lockstep (`run_tb.sh`) | Done |
| 5 | BD swap (`CartpoleDriverZynq_AXIS_Zedboard.tcl`), firmware link + backend switch | Done (bitstream rebuild pending on target machine) |
| 6 | On-rig shadow campaign → promote; measurements | **Remaining** |

---

## 7. Open points

- **`secloc_logic.c` include path in HLS**: the HLS project must reference
  `Firmware/Src/General/` (relative include or a sync step) so there is
  exactly one gate source. The tick-based entry point refactor must keep the
  existing firmware/PC-ctypes/parity-test builds green.
- **Vitis HLS 2020.1 float compliance**: add/mul/div are IEEE-compliant by
  default, but confirm in cosim that no `-ffast-math`-style config option is
  active in the project settings; the csim/cosim parity run is the proof.
- **Stream widths**: 12-bit codes ride in 32-bit AXIS beats (matches
  `controller_axis` expectations); a 2-output network (F1T) changes the
  response loop bound — same one-line package/loop edit as before.
- **`log_base` etc. as floats**: no clamping needed anymore (no milli-integer
  encoding); parameters are the same float32 values the C gate uses.
- **Old shells**: `controller_axilite`/`controller_axi` remain available for
  a plain-NN-only bitstream; always-fire mode on the gate block makes that
  mostly unnecessary.
- **10 MHz FCLK0**: ~15–30 µs of AXI traffic per step is fine; do the
  100 MHz move before quoting sub-µs wall-clock numbers (cycle *counts* from
  `nn_wait_cycles` and HLS reports are clock-independent).

---

## 8. Implementation status (2026-07-10)

Phases 1–5 are implemented in-tree; Phase 6 (on-rig shadow campaign) remains.

**Shared gate source.** `secloc_logic.c/.h` tick-based API
(`secloc_should_sample_tick`, etc.); C↔Python parity tests pass unmodified.

**Three HLS IPs** (`FPGA/CustomIPs/`):

| IP | Key files | csynth notes |
|---|---|---|
| `secloc_shell` | `secloc_shell_hls/secloc_shell.cpp` | ~29 cycles; AXI-Lite ↔ 14/4-word packets |
| `secloc_gate` | `secloc_gate_hls/secloc_gate.cpp` | Gate `INLINE off`; skip ~28 cyc, fire+watchdog up to 4221 cyc |
| `nn_marshal` | `nn_marshal_hls/nn_marshal.cpp` | Per-network via `nn_marshal_config.h`; re-synth on swap |

Protocol: `secloc_stream_protocol.h` (`SECLOC_STREAM_VERSION = 2`).
Testbenches: `secloc_shell_hls/run_tb.sh` (shell, marshal, three-block chain).
Build: `build_secloc_chain.sh`. Network swap: `swap_nn_and_build.sh`.

**Per-network config.** `FPGA/scripts/generate_nn_marshal_config.py` +
`post_hls4ml_marshal_config.py` hook in `Convert_Network_With_hls4ml.py`.
Example: `FPGA/NeuralNetworks/hls4ml_dense_1out_8_07_07_2026/nn_marshal_config.h`.

**Firmware.** `secloc_frontend_link.c/.h` (MMIO driver to shell registers,
`SFE_EXPECTED_VERSION = 2`); `hw_platform_config.h` detects
`SECLOC_SHELL_0` (legacy `SECLOC_FRONTEND_0` fallback);
`secloc_controller.c` (SW core) + `secloc_controller_pl.c` (PL backend
module) with backends `SW / PL / PL_SHADOW` and zero-force faults instead of
SW fallback; `neural_imitator.c` delegates in always-fire mode when
appropriate.

**Block design.** `CartpoleDriverZynq_AXIS_Zedboard.tcl` instantiates
`SECLOC_SHELL_0` + `SECLOC_GATE_0` + `NN_MARSHAL_0` + `CONTROLLER_AXIS_0`
at `0x40410000` (shell only). Rebuild scripts:
`rebuild_secloc_three_block.tcl`, `refresh_ip_and_build.tcl`,
`swap_nn_and_build.sh`.

**Retired.** Monolithic `secloc_frontend_hls` removed entirely (sources and
generated `solution1/` artifacts); see `FPGA/CustomIPs/README_secloc_chain.md`
for the replacement chain.
