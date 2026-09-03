# Reproducible model examples

Preserved controller bundles: training provenance, normalization, deployment
parameters, checksums, and physical validation data.

Full deployment context: [Docs/architecture.md](../Docs/architecture.md),
[Docs/operating.md](../Docs/operating.md).

## Show mux and PC paths

| Path | Model / controller | Weights / source | Documentation |
|---|---|---|---|
| **SW0** | AMP RPGD (CPU1) | RPGD worker blob in AMP CPU0 ELF | [RPGD paper](https://www.zora.uzh.ch/id/eprint/254218/1/RPGD_ICRA_2023.pdf); build via [Firmware/README.md](../Firmware/README.md) |
| **SW1** | Dense-7IN-32H1-32H2-1OUT-8 | `Firmware/Src/General/NC_C/` | Trained in CartPoleSimulation; C from SI_Toolkit export |
| **SW2** | LSTM-7IN-64H1-64H2-1OUT-0 | `Firmware/Src/General/NC_LSTM/` | [adaptive-quant-lstm-2025-06-01](adaptive-quant-lstm-2025-06-01/README.md) |
| **SW3** | Short-pole PL neural imitator | FPGA bitstream + [neural_imitator.c](../Firmware/Src/Zynq/neural_imitator.c) | Experiment-29 / hls4ml path in CartPoleSimulation |
| PC **`k`** `neural-imitator` | From `config_controllers.yml` | Active: `Dense-7IN-32H1-32H2-1OUT-1` (Exp-29) | [CartPoleSimulation README](../Driver/CartPoleSimulation/README.md) |
| PC **`k`** `mpc` + `rpgd-c` | Optimizer in Control Toolkit | N/A (online MPC) | Same as SW0 algorithm family on host |

After changing weights, rebuild/reflash firmware for on-chip paths. Align
`MOTOR_CORRECTION`, hanging, and angle circle in `globals.py` and
`parameters.c` — see [calibration.md](../Docs/calibration.md).

## Bundles in this folder

- [`adaptive-quant-lstm-2025-06-01`](adaptive-quant-lstm-2025-06-01/README.md):
  swing-up LSTM for PC TensorFlow and Zynq PS/C (`NC_LSTM`).

Add new bundles here with a README, checksums, and validation CSVs when a model
becomes a lab reference.
