# Hardware test protocol for the new XADC angle filter block

This document is self-contained: an agent following it step by step can guide the
operator through the whole campaign without further context.

## What is being tested

The FPGA block `median_filter` (HLS IP, `FPGA/CustomIPs/median_filter_hls/`) that
filters the XADC angle sensor stream (~450 kS/s). It was upgraded from a fixed
63-sample median to a runtime-configurable filter:

- `filter_mode`: 0 = raw passthrough, 1 = median, 2 = trimmed mean
  (trim_count samples cut from each end of the sorted window, rest averaged;
  trim_count = 0 gives a pure average)
- `window_size`: 1–64 (63 used by default)
- Boot default: trimmed mean, window 63, trim 7 (chosen from the 2026-07-08
  hardware campaign; the old median 63 remains available at runtime)
- The block exposes two AXI-Lite read registers: `filtered_o` (filter output)
  and `raw_o` (latest unfiltered XADC word), both full 16-bit
  (12-bit ADC code left-aligned = multiplied by 16; the filtered value carries
  ~4 fractional bits gained by averaging)
- **Dead-zone (rail) tracking** (added after the first campaign): the block
  compares every raw XADC sample against two thresholds (`rail_low` = 20x16,
  `rail_high` = 4090x16, set by firmware at init) and exposes five read
  registers: `dz_status_o` (bit0 latest sample at low rail, bit1 at high rail),
  `dz_window_o` (how many samples of the current filter window are near a
  rail, 0..window_size), `dz_age_o` (XADC samples, ~2.2 µs each, since the
  last rail contact; saturates at 0xFFFF), `dz_low_count` / `dz_high_count`
  (cumulative rail-hit counters since boot). Because the hardware checks every
  ~2.2 µs conversion, it also catches rail contacts that fall *between* the
  samples the firmware/PC records.

Goals, in order of importance:

1. Confirm the block works on real hardware in all modes (no stale data, no
   corruption, config changes apply live).
2. Quantify the noise/resolution improvement of trimmed mean vs the old median
   vs raw, on the real sensor.
3. Confirm outlier/glitch suppression is preserved (median's main virtue).
4. Confirm nothing regresses when the normal controller runs.

## Test infrastructure (already implemented, temporary test additions marked)

- Firmware serial command `CMD_SET_ANGLE_FILTER` (0xD2): reconfigures the
  filter live, echoes the config back. *Test addition; harmless to keep.*
- Firmware `CMD_COLLECT_RAW_ANGLE` extended with format byte: format 1 records
  pairs (filtered16, raw16) read back-to-back, up to 16384 pairs, motor stopped
  and control loop suspended during collection. Legacy format 0 unchanged.
  *Test addition; harmless to keep. Also fixes a pre-existing buffer overflow:
  collections used to be written into a 200-byte buffer.*
- Firmware `CMD_COLLECT_RAW_ANGLE` format 2 records 8-byte tuples
  (filtered16, raw16, dz_window u8, dz_status u8, dz_age u16). Firmware
  `CMD_GET_DEAD_ZONE` (0xD3) returns a snapshot of the dead-zone registers
  including the cumulative counters.
- Python: `Interface.set_angle_filter()`, `Interface.collect_angle_pairs()`,
  `Interface.collect_angle_deadzone()` and `Interface.get_dead_zone()`
  in `Driver/DriverFunctions/interface.py`.
- `run_filter_experiment.py` (this folder): phases `check`, `static`, `dynamic`,
  `deadzone`, `set`.
- `analyze_filter_experiment.py` (this folder): offline analysis, no hardware.
- `filter_reference_model.py`: bit-exact Python model of the HLS code (verified
  0/3000 mismatches against the C++ implementation for 10 configurations).

Key design idea: every recording contains the **raw stream alongside the
filtered one**, so any filter configuration can be replayed offline on the same
physical excitation. Physical repeatability is therefore NOT required — one
swing gives comparable data for all configurations.

## Prerequisites (verify before starting)

1. The Zedboard bitstream contains the new filter IP. It was rebuilt and
   exported in the previous session (`FPGA/VivadoProjects/.../cartpole_zedboard.xsa`,
   `BITSTREAM OK`). If in doubt, rebuild is NOT needed — just confirm the board
   is programmed with the current bitstream when the firmware is launched.
2. The firmware ELF is already built with the new commands
   (`Firmware/VitisProjects/CartPoleFirmware/Debug/CartPoleFirmware.elf`,
   contains symbols `cmd_SetAngleFilter`, `Goniometer_ReadPair16`). If sources
   change, rebuild in the Vitis GUI (project `CartPoleFirmware`) or run
   `make all` in `Firmware/VitisProjects/CartPoleFirmware/Debug` with Vitis
   2020.1 `settings64.sh` sourced.
3. Python environment: the one normally used for `Driver` — on this machine the
   conda env `cartpole` (`conda activate cartpole`, or call
   `~/anaconda3/envs/cartpole/bin/python` directly). Needs numpy, matplotlib,
   pyserial; `globals.py` provides port/baud settings. The plain system
   `python3` lacks pyserial and will fail.
4. USB serial cable connected; no other program (PhysicalCartPoleDriver GUI)
   holding the port.

## Step-by-step procedure

Operator actions are marked **[OPERATOR]**; everything else runs from the
terminal in the repo root. Total operator involvement: programming the board,
~4 swing releases, and 2 controller runs.

### Step 0 — Program the board (1 operator action)

**[OPERATOR]** Power the cartpole, then program FPGA + firmware the usual way
(Vitis GUI: Run As → Launch on Hardware for `CartPoleFirmware`, which also
programs the bitstream; or the SD-card BOOT.BIN flow via
`CartPoleFirmware_system`). The pole should hang freely; cart position is
irrelevant. No calibration is needed for any phase of this protocol.

### Step 1 — Sanity check (no operator action)

```bash
python Driver/DataAnalysis/HardwareFilterTest/run_filter_experiment.py check
```

Takes ~30 s. Pass criteria (script prints CHECK PASSED/FAILED):

- In raw mode, filtered output equals raw output for >80% of pairs (the two
  registers are read a few hundred ns apart; the filter updates every ~2.2 µs,
  so a small fraction may straddle an update).
- Pure average of 63 reduces noise std vs raw by at least 1.5x (expect ~4–8x).

If this fails, stop and debug: most likely the board is running an old
bitstream/firmware combination. Symptoms: `set_angle_filter` echo timeout means
old firmware; noise reduction failure means the IP did not get the new
configuration registers.

### Step 2 — Static noise sweep (no operator action, pole must hang still)

**[OPERATOR]** Confirm the pole hangs still and nothing touches the rig
(avoid desk vibrations for ~1 minute).

```bash
python Driver/DataAnalysis/HardwareFilterTest/run_filter_experiment.py static
```

Sweeps 10 configurations x 10000 pairs at 100 µs (~1 min), saves
`output/static_sweep_<timestamp>.npz`. The sweep covers: raw, median 63 (old
design), pure average 63, trimmed 63 with trim 3/7/15/31, median 31,
trimmed 31/3, trimmed 15/1.

### Step 3 — Dynamic swing recordings (1 operator action per repetition, default 3)

```bash
python Driver/DataAnalysis/HardwareFilterTest/run_filter_experiment.py dynamic --repetitions 3
```

For each repetition the script prompts:

**[OPERATOR]** Lift the pole to roughly horizontal, hold it still, press Enter,
release exactly on "GO" (3-second countdown), then hands off. Each recording is
16384 pairs at 500 µs ≈ 8 s of free damped oscillation. Starting angle does not
need to be reproducible — raw is recorded simultaneously, so all filter
comparisons happen on identical data.

The script warns if the recorded angle span is suspiciously small (missed
release). Repeat that repetition if warned.

### Step 3b — Dead-zone recordings (1 operator action per repetition, default 3)

Tests the hardware rail detection at the potentiometer dead zone (the ~17°
gap in the resistive track, located to the side of the swing range).

```bash
python Driver/DataAnalysis/HardwareFilterTest/run_filter_experiment.py deadzone --repetitions 3
```

No precise release is required. The physics does the work: a decaying free
swing released above the dead-zone side first *crosses* the zone, then — as
the amplitude shrinks — inevitably passes through a band where it *turns
around inside* the zone, and finally stops reaching it. One release therefore
produces crossings, turnarounds, and near-misses in a single recording.

**[OPERATOR]** Per repetition (3-second countdown as in the dynamic phase):
lift the pole a moderate amount above the dead-zone side — high enough that
the first swings pass through the zone, but not much higher, so the
turnaround band is reached within the ~16 s recording. Release on "GO" and
hands off. The script prints immediately whether the rails were touched and
warns if the pole never reached the zone (release higher and repeat).

Each repetition records 16384 tuples at 1000 µs (~16 s) and saves
`output/deadzone_swing_<timestamp>_rep<N>.npz`. The analyzer classifies each
rail episode as CROSSING or TURNAROUND automatically from the on-track signal
before/after the episode, so no per-maneuver bookkeeping is needed.

### Step 4 — Offline analysis (no hardware, no operator)

```bash
python Driver/DataAnalysis/HardwareFilterTest/analyze_filter_experiment.py
```

Analyzes everything in `output/`, prints tables, writes `*_analysis.png` next to
each recording. Read the "Expected results" section below to judge them.

### Step 5 (optional) — Controller-in-the-loop A/B check (2 operator actions)

Only needed as a final regression check; skip if steps 1–4 are all green and
time is short. The filter config persists across serial reconnects (it resets
only on board power cycle / firmware restart), so the flow is:

1. Set the OLD behavior (median 63):

```bash
python Driver/DataAnalysis/HardwareFilterTest/run_filter_experiment.py set --window 63 --trim 0 --mode 1
```

2. **[OPERATOR]** Start the normal control software
   (`python Driver/control.py` or the usual entry point), calibrate, run the
   standard stabilization for ~60 s with recording enabled, stop and close the
   program (it must release the serial port).
3. Set the NEW default (trimmed mean 63, trim 7):

```bash
python Driver/DataAnalysis/HardwareFilterTest/run_filter_experiment.py set --window 63 --trim 7 --mode 2
```

4. **[OPERATOR]** Repeat the same controller run.
5. Compare the two CSVs in `Driver/ExperimentRecordings/` (columns `angle_raw`,
   `angleD_raw`/`angleD`, `Q`): with the trimmed mean expect equal or lower
   noise in `angle_raw` residuals and `angleD`, and no increase in invalid
   steps. Runs are not identical (different starting conditions) — compare
   noise statistics during stable stabilization segments, not trajectories.

Important: do NOT run the controller in raw mode (mode 0) — unfiltered glitches
can exceed the firmware's invalid-step threshold (`MAX_ADC_STEP` = 20) and
degrade control.

### Step 6 — Cleanup

- A board power cycle (or firmware restart) restores the boot default
  (trimmed 63/7); the `static`/`dynamic`/`deadzone` phases also restore it on exit.
- Recordings and plots live in this folder's `output/` (gitignored via *.png;
  .npz files are small enough to keep or delete at will).
- The firmware/driver additions are designed to be permanently useful
  (runtime filter reconfiguration + paired collection) and need not be
  reverted.

## Expected results / pass criteria

Units: recordings are 16-bit codes; divide by 16 for 12-bit ADC LSB.

**Static sweep table:**

- Raw noise std: whatever the sensor gives (historically ~1–3 LSB); this is the
  baseline.
- Median 63 (old design) and trimmed 63/7 (new default): both should cut noise
  strongly; expect roughly 2–3 "bits gained" (log2 of std ratio). Trimmed mean
  should be equal or better than median if the noise is near-Gaussian; if the
  ADC noise is heavy-tailed, they should be close.
- Pure average 63: best or near-best std on glitch-free data, but the point of
  trimming is robustness, not winning this row.
- Glitch column: 0 for all filtered configurations. Raw may show occasional
  glitches — that is exactly the phenomenon the filter must suppress. (If raw
  shows zero glitches across all recordings, the historical glitch concern may
  simply not reproduce; note that in the report.)
- "model dev" (hardware vs bit-exact Python model replayed on the recorded raw
  stream): well below one 12-bit LSB (16 units) for every configuration. Large
  deviations mean the hardware is not computing what the model computes —
  investigate before trusting anything else.
- The filtered streams should show values *between* adjacent multiples of 16 —
  that is the sub-LSB resolution gained by averaging (the old median could only
  output multiples of 16 divided down to integer codes).

**Dynamic recordings:**

- Measured filtered-vs-raw lag: 0 recording samples (hardware group delay is
  ~70 µs for window 63, far below the 500 µs recording interval). A lag of
  several samples would indicate stale data in the pipeline — a critical bug.
- Hardware filtered glitch count: 0 (raw may have some).
- Derivative std: hardware filtered and replayed trimmed 63/7 clearly below
  raw; trimmed configurations at or below median.
- Full-recording plot: filtered curve lies on top of raw with no visible
  offset, overshoot, or step artifacts through the whole decay.

**Dead-zone recordings:**

- `dz_status vs thresholds on recorded raw: 0 mismatches` — the hardware flags
  must agree exactly with applying the thresholds to the recorded raw samples.
  Any mismatch is a hardware bug.
- Each recording should contain several episodes: `CROSSING` ones early
  (on-track signal on opposite sides before/after the episode) and
  `TURNAROUND` ones later in the decay (same side before and after). Both
  cumulative counters (low and high) should increase over a recording with
  crossings — during a crossing the input rails high at the gap edge and then
  decays through the low rail (RC discharge of the floating ADC node,
  ~1.5 ms). NOTE: which *rails* were touched does not discriminate crossing
  from turnaround — the electrical decay to the low rail happens whichever
  way the pole leaves — which is exactly why the analyzer classifies by the
  on-track signal around the episode, not by the rails.
- If no repetition contains a TURNAROUND episode, the release was too high
  (decay did not reach the turnaround band within the recording) — repeat
  with a lower release; no precision is needed, only "lower".
- `dz_window` should ramp up to ~window_size during a dwell in the zone and
  ramp back to 0 within one window length (~140 µs at 63 samples) after
  leaving — this is the signal the firmware can use to know the filtered
  value is contaminated.
- `dz_age` should stay 0 throughout an episode even at recording gaps —
  hardware sees every 2.2 µs conversion, so brief rail contacts between
  recorded samples are still registered.

**Controller A/B (if run):** no degradation in stabilization quality or invalid
step count with the new default vs the old median.

## Troubleshooting

- `set_angle_filter` echo timeout / RuntimeError: firmware without the new
  commands, or port occupied. Rebuild+reflash firmware (see prerequisites).
- `collect_angle_pairs` timeout: usually the board went through calibration or
  is in control mode; power cycle the board and rerun. The script disables
  control mode itself, so a persistent timeout points at a serial issue.
- All filtered values identical to raw in every mode: bitstream is old
  (block ignores mode register). Reprogram FPGA with the current bitstream.
- Static sweep std ~0 for raw: sensor unplugged or XADC channel mis-mapped;
  check the goniometer cable.
- Serial garbage after an aborted collection: unplug/replug is not needed —
  just rerun the script (it drains and resyncs on the SOF byte).
