-- =====================================================================
-- Controller I/O parameters shared by controller_* shells and adapters.
--
-- SHORT POLE BUILD (hls4ml_short_pole): 7 x 14-bit in, 14-bit out.
-- =====================================================================
library ieee;
use ieee.std_logic_1164.all;

package controller_io_parameters is
    constant CONTROLLER_INPUTS           : integer := 7;
    constant BITS_PER_CONTROLLER_INPUT   : integer := 14;

    constant CONTROLLER_OUTPUTS          : integer := 1;
    constant BITS_PER_CONTROLLER_OUTPUT  : integer := 14;
end package controller_io_parameters;
