# Adaptive physical-cartpole LSTM (2025-06-01)

This is the preserved, runnable model bundle for the physical-cartpole adaptive
swing-up controller:

`Long/quant/LSTM-7IN-64H1-64H2-1OUT-0`

It is the model selected by the late 3 June 2025 configuration and recovered as
the working PC/TensorFlow and Zynq PS/C controller in August 2026. Parent-repo
commit `3bab8a20` is the August working firmware/configuration reference;
`feaaa124` contains the initial side-by-side Zynq PS integration. After the
2026-09-02 potentiometer remesure, the go-to physical reference is the
`WORKING REFERENCE: PC and on-chip LSTM` commit on `Development` (hanging
3261.643, circle 4044.15).

## What “quant” means here

`quant` is the historical folder name. The training configuration has
`QUANTIZATION.ACTIVATED: false`; the Keras model and generated firmware arrays
use float32 weights.

## Network

- Framework: TensorFlow/Keras 2.14
- Architecture: LSTM(64, sequences) → LSTM(64, sequences) → Dense(1, linear)
- Parameters: 51,521
- Training input shape: `(batch, 110, 7)`
- Runtime recurrence: one stateful step every 10 ms
- Input order:
  `angleD, angle_cos, angle_sin, position, positionD, target_equilibrium, target_position`
- Output: normalized motor command `Q`
- Training washout: 100 samples
- Label shift: one sample
- Training seed: 1873

Input normalization multiplies the seven inputs by:

```text
0.00332930, 1, 1, 5.05050516, 0.18420650, 1, 5.05050516
```

All normalization offsets are zero. Output denormalization is the identity.

## Origin

- Trained on 1 June 2025.
- Recorded source revision:
  `ea1d585945e947e00a83772fb21bbdbdecf04ae1`
  (`origin/Experiment_4_05_2025` in the CartPoleSimulation repository).
- Normalization generated at 17:53:28.
- Model metadata created at 18:12:20.
- Keras model and TensorFlow checkpoint saved at 20:27:53.
- First tracked controller selection:
  CartPoleSimulation commit `b9c6cfae`, 3 June 2025.
- Original ignored source:
  `Driver/CartPoleSimulation/SI_Toolkit_ASF/Experiments/`
  `MPC_before_NNI_6_05_s1_1t_qunatized_cleaned/Models/`
  `LSTM-7IN-64H1-64H2-1OUT-0`.
- Historical deployment copy:
  `Driver/CartPoleSimulation/SI_Toolkit_ASF/Experiments/`
  `CP_Models_02_06_2025/Long/quant/LSTM-7IN-64H1-64H2-1OUT-0`.

The files in `models/LSTM-7IN-64H1-64H2-1OUT-0/` preserve the complete
historical model bundle. Fifteen files are byte-identical to the June source.
`ckpt.weights.h5` is a later convenience export; all eight learned tensors in
it are bit-identical to the tensors in the `.keras` archive.

The original archive was written by Keras 2.14 and does not load natively under
the current CPP39 environment's TensorFlow/Keras 2.13. The repository's normal
neural evaluator therefore reconstructs the architecture and loads
`ckpt.weights.h5`. That application path was verified from this example
directory; the original `.keras` archive remains preserved for provenance and
use with its native framework version.

## Training data

The model references 234 CSV recordings:

- 221 training recordings, 481,423,407 bytes
- 6 validation recordings, 31,690,899 bytes
- 7 test recordings, 27,757,252 bytes
- Total: 540,871,558 bytes

Those ignored raw recordings are intentionally not duplicated into ordinary
Git. `data/TRAINING_DATA_SHA256SUMS` records every source-relative path, byte
size, and SHA-256 digest, so the exact corpus can be identified and verified.
Use Git LFS or an external immutable data archive before removing the existing
local source corpus.

## Physical deployment

The working reference uses:

- 10 ms control period
- two-sample derivative window (`N=2`, spanning 20 ms)
- FPGA median angle filter, window 63
- hanging angle 3261.643 ADC counts (remeasured 2026-09-02; August ref used 1014)
- full angle span 4044.15 ADC counts (August ref used 4066.88)
- motor map `(0.5733488, 0.0257380, 0.0258429)`
- clipped network output in `[-1, 1]`
- zero recurrent state followed by one nominal-upright priming step
- FPGA rail-event detection plus bounded June-style 45/90 ms gap extrapolation

The generated pure-C tensors are tracked in
`Firmware/Src/General/NC_LSTM/network_parameters.c`. The eight learned arrays
match the Keras float32 tensors exactly. Offline C/TensorFlow sequence parity
was verified to a maximum output difference of approximately `3.2e-6`.

After every cartpole power cycle, run position calibration (`Shift+K`) before
enabling control. `python Driver/control.py` configures the FTDI timer and sends
the runtime derivative/filter settings. Use `k` for PC/TensorFlow control or
`u` for Zynq PS/C control. The two paths implement the same policy, but the PC
path remains subject to host/serial timing and computes cart velocity on the PC.

## Validation recordings

`validation/CPP_neural-imitator_2026-08-31_17-26-53.csv` is the gap-corrected
normal-driver run used to validate the restored behavior. It used the preceding
`N=1` setting: sustained capture began after about 8.17 seconds, remained within
3 degrees for 66.82 seconds, and had no corrected raw angular-derivative sample
above 85 ADC counts per control step.

`validation/CPP_zynq-ps-lstm-primed-retry_2026-08-31_11-45-50.csv` is the
earlier PS baseline before the final gap fix. It contains derivative corruption
up to 2027 ADC counts per step and required about 27.31 seconds to reach its
long sustained upright interval. See `validation/metrics.yml`.

The final `N=2` selection was made after a subsequent physical comparison and
is represented by the working source configuration, but that manually observed
run was not recorded.

On 2026-09-02, after remesuring hang/upright on the same Zybo, PC
`neural-imitator` balanced from hanging for 167 s (157 s with |angle|<0.3).
Zynq PS LSTM (`OnChipController_neural_controller_LSTM_C`) swung up in 9.1 s
and ran 120 s (96 s near upright) while the JB slider moved the target. Host C
and TensorFlow match to about `1e-6` on the same inputs.

## Integrity

From this directory:

```console
sha256sum -c SHA256SUMS
```

`provenance.yml` contains the same identity and deployment facts in a
machine-readable form.
