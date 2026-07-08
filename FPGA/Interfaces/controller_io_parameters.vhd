-- =====================================================================
-- Controller I/O parameters shared by all controller_* interface shells
-- (controller_axi / controller_axilite / controller_axis) and the
-- controller_adapter_* wrappers.
--
-- Reconstructed for the repository: this package was previously only
-- present inside the (gitignored) Vivado project directory.
--
-- Values must match BOTH:
--   * the hls4ml core entity `myproject`
--       input_1_V      : 84 bits = CONTROLLER_INPUTS * BITS_PER_CONTROLLER_INPUT
--       layer9_out_0_V : 12 bits = CONTROLLER_OUTPUTS * BITS_PER_CONTROLLER_OUTPUT
--     (FPGA/NeuralNetworks/hls4ml_dense_1out_8_07_07_2026/myproject.vhd)
--   * the firmware (Firmware/Src/Zynq/neural_imitator.h):
--       MLP_ACTIVATION_NEURONS      = 7
--       MLP_PREDICTION_NEURONS      = 1
--       MLP_TOTAL_BITS_PER_VARIABLE = 12
-- =====================================================================
library ieee;
use ieee.std_logic_1164.all;

package controller_io_parameters is
    constant CONTROLLER_INPUTS           : integer := 7;
    constant BITS_PER_CONTROLLER_INPUT   : integer := 12;

    constant CONTROLLER_OUTPUTS          : integer := 1;
    constant BITS_PER_CONTROLLER_OUTPUT  : integer := 12;
end package controller_io_parameters;
