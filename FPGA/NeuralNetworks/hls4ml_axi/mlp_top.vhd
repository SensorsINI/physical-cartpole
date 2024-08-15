----------------------------------------------------------------
--          Top module for MLP 1 Neuron hls module            --
--  This module has AXIStream interface for input and output. --
--  Input AXIStream is translate to the MLP input format.     --
--  Output MLP format is translated to AXIStream.             -- 
----------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;                           -- @suppress "Superfluous library clause: access to library 'work' is implicit"
use work.mlp_top_pkg.all;

entity mlp is
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
        M_AXIS_TREADY : in  std_logic
    );
end entity mlp;

architecture RTL of mlp is
    -- Signal declaration
    signal ap_rst                             : STD_LOGIC;
    signal ap_start                           : STD_LOGIC;
    signal ap_done                            : STD_LOGIC;
    signal ap_idle                            : STD_LOGIC;
    signal ap_ready_r, ap_ready               : STD_LOGIC;
    signal activation_ap_valid                : STD_LOGIC;
    signal activation_data_r, activation_data : STD_LOGIC_VECTOR(MLP_INPUT_BIT_WIDTH - 1 downto 0);
    signal prediction_data                    : STD_LOGIC_VECTOR(MLP_OUTPUT_BIT_WIDTH - 1 downto 0);
    signal prediction_ap_valid                : STD_LOGIC; -- @suppress "signal prediction_ap_valid is never read"
    signal const_size_in_1                    : STD_LOGIC_VECTOR(15 downto 0); -- @suppress "signal const_size_in_1 is never read"
    signal const_size_in_1_ap_vld             : STD_LOGIC; -- @suppress "signal const_size_in_1_ap_vld is never read"
    signal const_size_out_1                   : STD_LOGIC_VECTOR(15 downto 0); -- @suppress "signal const_size_out_1 is never read"
    signal const_size_out_1_ap_vld            : STD_LOGIC; -- @suppress "signal const_size_out_1_ap_vld is never read"
    signal read_inputs_r, read_inputs         : integer := 0;
    signal write_outputs_r, write_outputs     : integer := 0;

    type state is (idle, read_axi_inputs, feed_mlp, wait_mlp_ready, wait_mlp_done, write_axi_outputs);
    signal cs, ns : state;

begin

    ap_rst        <= not AXI_ARESETN;
    S_AXIS_TREADY <= ap_idle;

    -- State machine
    b_sync : process(AXIS_ACLK, AXI_ARESETN)
    begin
        if AXI_ARESETN = '0' then
            cs                <= idle;
            read_inputs_r     <= 0;
            write_outputs_r   <= 0;
            activation_data_r <= (others => '0');
            ap_ready_r        <= '0';

        elsif rising_edge(AXIS_ACLK) then
            cs                <= ns;
            read_inputs_r     <= read_inputs;
            write_outputs_r   <= write_outputs;
            activation_data_r <= activation_data;
            ap_ready_r        <= ap_ready;
        end if;
    end process b_sync;

    b_conv : process(cs, S_AXIS_TVALID, S_AXIS_TLAST, S_AXIS_TDATA, activation_data_r, read_inputs_r, write_outputs_r, ap_done, ap_ready_r, prediction_data, M_AXIS_TREADY)
    begin
        ns                  <= cs;
        read_inputs         <= read_inputs_r;
        write_outputs       <= write_outputs_r;
        activation_data     <= activation_data_r;
        activation_ap_valid <= '0';
        M_AXIS_TLAST        <= '0';
        M_AXIS_TDATA        <= (others => '0');
        M_AXIS_TVALID       <= '0';
        ap_start            <= '0';
        case cs is
            when idle =>
                if S_AXIS_TVALID = '1' then
                    -- Read the first input data and move to read_axi_inputs state
                    activation_data(MLP_INPUT_DATA_BITS - 1 downto 0) <= S_AXIS_TDATA(MLP_INPUT_DATA_BITS - 1 downto 0);
                    read_inputs                                       <= read_inputs_r + 1;
                    -- Check if it is the last input data
                    if S_AXIS_TLAST = '1' then
                        -- Move to wait_mlp_ready state
                        ns          <= feed_mlp;
                        read_inputs <= 0;
                    else
                        -- Move to read_axi_inputs state
                        ns <= read_axi_inputs;
                    end if;
                end if;

            when read_axi_inputs =>
                if S_AXIS_TVALID = '1' then
                    -- Read the next input data
                    activation_data((MLP_INPUT_DATA_BITS * read_inputs_r) + MLP_INPUT_DATA_BITS - 1 downto (MLP_INPUT_DATA_BITS * read_inputs_r)) <= S_AXIS_TDATA(MLP_INPUT_DATA_BITS - 1 downto 0);
                    read_inputs                                                                                                                   <= read_inputs_r + 1;
                    -- Check if it is the last input data
                    if S_AXIS_TLAST = '1' then
                        -- Move to wait_mlp_ready state
                        ns          <= feed_mlp;
                        read_inputs <= 0;
                    end if;
                end if;

            when feed_mlp =>
                -- Feed the MLP with the input data
                activation_ap_valid <= '1';
                ap_start            <= '1';
                -- Move to wait_mlp_done state
                ns                  <= wait_mlp_ready;

            when wait_mlp_ready =>
                if ap_ready_r = '0' then
                    -- Remains ap_start until ap_ready = '1'
                    ap_start <= '1';
                else
                    -- Move to wait_mlp_done state
                    ns <= wait_mlp_done;
                end if;

            when wait_mlp_done =>
                if ap_done = '1' then
                    -- Move to write_axi_outputs state
                    ns <= write_axi_outputs;
                end if;

            when write_axi_outputs =>
                if M_AXIS_TREADY = '1' then
                    -- Write the first output data
                    M_AXIS_TDATA(MLP_OUTPUT_DATA_BITS - 1 downto 0) <= prediction_data((MLP_OUTPUT_DATA_BITS * read_inputs_r) + MLP_OUTPUT_DATA_BITS - 1 downto (MLP_OUTPUT_DATA_BITS * write_outputs_r));
                    write_outputs                                   <= write_outputs_r + 1;
                    M_AXIS_TVALID                                   <= '1';
                    -- Check if it is the last output data
                    if write_outputs_r = MLP_OUTPUT_NEURONS - 1 then
                        -- Move to idle state
                        ns            <= idle;
                        write_outputs <= 0;
                        M_AXIS_TLAST  <= '1';
                    end if;
                end if;

        end case;
    end process b_conv;

    -- Instantiate the MLP 1 Neuron hls module
    inst_mlp : entity work.myproject
        port map(
            ap_clk                  => AXIS_ACLK,
            ap_rst                  => ap_rst,
            ap_start                => ap_start,
            ap_done                 => ap_done,
            ap_idle                 => ap_idle,
            ap_ready                => ap_ready,
            input_1_V               => activation_data,
            input_1_V_ap_vld        => activation_ap_valid,
            layer9_out_0_V         => prediction_data,
            layer9_out_0_V_ap_vld  => prediction_ap_valid
        );

end architecture RTL;

