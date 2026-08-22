----------------------------------------------------------------
--          AXI Stream interface for controller              --
--  This module has AXIStream interface for input and output. --
--  Input AXIStream is translated to the controller format.   --
--  Output controller format is translated to AXIStream.      -- 
----------------------------------------------------------------
--
-- =====================================================================
-- *** CONFIGURATION: Select HLS Core Adapter ***
-- =====================================================================
-- Change the entity name below to match your HLS core:
--   - controller_adapter_cp   : For 1-output CartPole cores (layer9_out_0_V)
--   - controller_adapter_f1t  : For 2-output F1T cores (layer8_out_0_V, layer8_out_1_V)
-- 
-- Edit the entity name on this line ONLY:
-- >>> CONTROLLER_ADAPTER_ENTITY: controller_adapter_cp <<<
--
-- Then update line ~169 in the architecture to match.
-- =====================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.controller_io_parameters.all;

entity controller_axis is
    generic(
        AXI_DATA_WIDTH : integer := 32
    );
    port(
        -- AXI clock and reset
        AXIS_ACLK     : in  std_logic;
        AXI_ARESETN   : in  std_logic;
        -- AXI input stream
        S_AXIS_TREADY : out std_logic;
        S_AXIS_TDATA  : in  std_logic_vector(AXI_DATA_WIDTH - 1 downto 0);
        S_AXIS_TLAST  : in  std_logic;
        S_AXIS_TVALID : in  std_logic;
        -- AXI output stream
        M_AXIS_TVALID : out std_logic;
        M_AXIS_TDATA  : out std_logic_vector(AXI_DATA_WIDTH - 1 downto 0);
        M_AXIS_TLAST  : out std_logic;
        -- Constant all-ones side channels for slaves that expect them
        -- (e.g. the Vitis HLS axis port of the SecLoc frontend IP).
        M_AXIS_TKEEP  : out std_logic_vector(AXI_DATA_WIDTH/8 - 1 downto 0);
        M_AXIS_TSTRB  : out std_logic_vector(AXI_DATA_WIDTH/8 - 1 downto 0);
        M_AXIS_TREADY : in  std_logic
    );
end entity controller_axis;

architecture RTL of controller_axis is
    -- Controller I/O parameters from package
    constant NUM_INPUTS        : integer := CONTROLLER_INPUTS;
    constant BITS_PER_INPUT    : integer := BITS_PER_CONTROLLER_INPUT;
    constant NUM_OUTPUTS       : integer := CONTROLLER_OUTPUTS;
    constant BITS_PER_OUTPUT   : integer := BITS_PER_CONTROLLER_OUTPUT;
    
    constant TOTAL_INPUT_BITS  : integer := NUM_INPUTS * BITS_PER_INPUT;
    constant TOTAL_OUTPUT_BITS : integer := NUM_OUTPUTS * BITS_PER_OUTPUT;

    -- Core adapter signals
    signal ap_rst                             : STD_LOGIC;
    signal ap_start                           : STD_LOGIC;
    signal ap_done                            : STD_LOGIC;
    signal ap_idle                            : STD_LOGIC;
    signal ap_ready_r, ap_ready               : STD_LOGIC;
    signal core_in_valid                      : STD_LOGIC;
    signal core_in_data_r, core_in_data       : STD_LOGIC_VECTOR(TOTAL_INPUT_BITS - 1 downto 0);
    signal core_out_data                      : STD_LOGIC_VECTOR(TOTAL_OUTPUT_BITS - 1 downto 0);
    signal core_out_valid                     : STD_LOGIC; -- @suppress "signal core_out_valid is never read"
    signal read_inputs_r, read_inputs         : integer := 0;
    signal write_outputs_r, write_outputs     : integer := 0;

    type state is (idle, read_axi_inputs, feed_controller, wait_controller_ready, wait_controller_done, write_axi_outputs);
    signal cs, ns : state;

begin

    ap_rst        <= not AXI_ARESETN;
    S_AXIS_TREADY <= ap_idle;
    M_AXIS_TKEEP  <= (others => '1');
    M_AXIS_TSTRB  <= (others => '1');

    -- State machine
    b_sync : process(AXIS_ACLK, AXI_ARESETN)
    begin
        if AXI_ARESETN = '0' then
            cs              <= idle;
            read_inputs_r   <= 0;
            write_outputs_r <= 0;
            core_in_data_r  <= (others => '0');
            ap_ready_r      <= '0';

        elsif rising_edge(AXIS_ACLK) then
            cs              <= ns;
            read_inputs_r   <= read_inputs;
            write_outputs_r <= write_outputs;
            core_in_data_r  <= core_in_data;
            ap_ready_r      <= ap_ready;
        end if;
    end process b_sync;

    b_conv : process(cs, S_AXIS_TVALID, S_AXIS_TLAST, S_AXIS_TDATA, core_in_data_r, read_inputs_r, write_outputs_r, ap_done, ap_ready_r, core_out_data, M_AXIS_TREADY)
    begin
        ns             <= cs;
        read_inputs    <= read_inputs_r;
        write_outputs  <= write_outputs_r;
        core_in_data   <= core_in_data_r;
        core_in_valid  <= '0';
        M_AXIS_TLAST   <= '0';
        M_AXIS_TDATA   <= (others => '0');
        M_AXIS_TVALID  <= '0';
        ap_start       <= '0';
        case cs is
            when idle =>
                if S_AXIS_TVALID = '1' then
                    -- Read the first input data and move to read_axi_inputs state
                    core_in_data(BITS_PER_INPUT - 1 downto 0) <= S_AXIS_TDATA(BITS_PER_INPUT - 1 downto 0);
                    read_inputs                               <= read_inputs_r + 1;
                    -- Check if it is the last input data
                    if S_AXIS_TLAST = '1' then
                        -- Move to wait_controller_ready state
                        ns          <= feed_controller;
                        read_inputs <= 0;
                    else
                        -- Move to read_axi_inputs state
                        ns <= read_axi_inputs;
                    end if;
                end if;

            when read_axi_inputs =>
                if S_AXIS_TVALID = '1' then
                    -- Read the next input data
                    core_in_data((BITS_PER_INPUT * read_inputs_r) + BITS_PER_INPUT - 1 downto (BITS_PER_INPUT * read_inputs_r)) <= S_AXIS_TDATA(BITS_PER_INPUT - 1 downto 0);
                    read_inputs                                                                                                  <= read_inputs_r + 1;
                    -- Check if it is the last input data
                    if S_AXIS_TLAST = '1' then
                        -- Move to wait_controller_ready state
                        ns          <= feed_controller;
                        read_inputs <= 0;
                    end if;
                end if;

            when feed_controller =>
                -- Feed the controller with the input data
                core_in_valid <= '1';
                ap_start      <= '1';
                -- Move to wait_controller_done state
                ns            <= wait_controller_ready;

            when wait_controller_ready =>
                if ap_ready_r = '0' then
                    -- Remains ap_start until ap_ready = '1'
                    ap_start <= '1';
                else
                    -- Move to wait_controller_done state
                    ns <= wait_controller_done;
                end if;

            when wait_controller_done =>
                if ap_done = '1' then
                    -- Move to write_axi_outputs state
                    ns <= write_axi_outputs;
                end if;

            when write_axi_outputs =>
                if M_AXIS_TREADY = '1' then
                    -- Write the first output data
                    M_AXIS_TDATA(BITS_PER_OUTPUT - 1 downto 0) <= core_out_data((BITS_PER_OUTPUT * write_outputs_r) + BITS_PER_OUTPUT - 1 downto (BITS_PER_OUTPUT * write_outputs_r));
                    write_outputs                              <= write_outputs_r + 1;
                    M_AXIS_TVALID                              <= '1';
                    -- Check if it is the last output data
                    if write_outputs_r = NUM_OUTPUTS - 1 then
                        -- Move to idle state
                        ns            <= idle;
                        write_outputs <= 0;
                        M_AXIS_TLAST  <= '1';
                    end if;
                end if;

        end case;
    end process b_conv;

    -- =====================================================================
    -- Controller adapter instance (wraps HLS core)
    -- =====================================================================
    -- *** TO CHANGE ADAPTER: Edit the entity name on the line below ***
    -- Current: controller_adapter_cp (1-output CartPole core)
    -- Options: controller_adapter_cp | controller_adapter_f1t
    -- See configuration section at top of file for details.
    -- =====================================================================
    inst_adapter: entity work.controller_adapter_cp
        port map(
            core_clk       => AXIS_ACLK,
            core_rst       => ap_rst,
            core_start     => ap_start,
            core_ready     => ap_ready,
            core_done      => ap_done,
            core_in        => core_in_data,
            core_in_valid  => core_in_valid,
            core_out       => core_out_data,
            core_out_valid => core_out_valid
        );
    
    -- ap_idle is high when the adapter is ready for a new transaction
    ap_idle <= not ap_start and not ap_done;

end architecture RTL;
