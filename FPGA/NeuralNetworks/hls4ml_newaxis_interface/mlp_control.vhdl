-- File: mlp_control.vhdl

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.mlp_top_pkg.all;

entity mlp_control is
    generic(
        AXI_DATA_WIDTH : integer := 32
    );
    port(
        clk              : in  std_logic;
        reset_n          : in  std_logic;
        
        -- Interface to AXI Stream Module
        data_in          : in  std_logic_vector(AXI_DATA_WIDTH-1 downto 0);
        data_in_valid    : in  std_logic;
        data_in_last     : in  std_logic;
        data_in_ready    : out std_logic;
        
        data_out         : out std_logic_vector(AXI_DATA_WIDTH-1 downto 0);
        data_out_valid   : out std_logic;
        data_out_last    : out std_logic;
        data_out_ready   : in  std_logic
    );
end entity mlp_control;

architecture RTL of mlp_control is
    -- Internal Signals
    signal ap_rst                             : STD_LOGIC;
    signal ap_start                           : STD_LOGIC;
    signal ap_done                            : STD_LOGIC;
    signal ap_idle                            : STD_LOGIC;
    signal ap_ready_r, ap_ready               : STD_LOGIC;
    signal activation_ap_valid                : STD_LOGIC;
    signal activation_data_r, activation_data : STD_LOGIC_VECTOR(MLP_INPUT_BIT_WIDTH - 1 downto 0);
    signal prediction_data                    : STD_LOGIC_VECTOR(MLP_OUTPUT_BIT_WIDTH - 1 downto 0);
    signal prediction_ap_valid                : STD_LOGIC; -- Not used in current design
    
    signal read_inputs_r, read_inputs         : integer := 0;
    signal write_outputs_r, write_outputs     : integer := 0;

    -- Type for state machine
    type state_type is (idle, read_axi_inputs, feed_mlp, wait_mlp_ready, wait_mlp_done, write_axi_outputs);
    signal cs, ns : state_type;

begin

    -- Reset Signal
    ap_rst <= not reset_n;

    -- State Machine Synchronization
    b_sync : process(clk, reset_n)
    begin
        if reset_n = '0' then
            cs                <= idle;
            read_inputs_r     <= 0;
            write_outputs_r   <= 0;
            activation_data_r <= (others => '0');
            ap_ready_r        <= '0';
        elsif rising_edge(clk) then
            cs                <= ns;
            read_inputs_r     <= read_inputs;
            write_outputs_r   <= write_outputs;
            activation_data_r <= activation_data;
            ap_ready_r        <= ap_ready;
        end if;
    end process b_sync;

    -- Control Logic
    b_conv : process(cs, data_in_valid, data_in_last, data_in, activation_data_r, read_inputs_r, write_outputs_r, ap_done, ap_ready_r, prediction_data, data_out_ready)
    begin
        -- Default assignments
        ns                  <= cs;
        read_inputs         <= read_inputs_r;
        write_outputs       <= write_outputs_r;
        activation_data     <= activation_data_r;
        activation_ap_valid <= '0';
        data_out_valid      <= '0';
        data_out_last       <= '0';
        ap_start            <= '0';
        data_out            <= (others => '0');

        case cs is
            when idle =>
                if data_in_valid = '1' then
                    -- Corrected: Assign a slice of activation_data based on read_inputs_r
                    activation_data((MLP_INPUT_DATA_BITS * read_inputs_r) + MLP_INPUT_DATA_BITS - 1 downto (MLP_INPUT_DATA_BITS * read_inputs_r)) <= data_in(MLP_INPUT_DATA_BITS - 1 downto 0);
                    read_inputs <= read_inputs_r + 1;
                    if data_in_last = '1' then
                        ns          <= feed_mlp;
                        read_inputs <= 0;
                    else
                        ns <= read_axi_inputs;
                    end if;
                end if;

            when read_axi_inputs =>
                if data_in_valid = '1' then
                    -- Corrected: Assign a slice of activation_data based on read_inputs_r
                    activation_data((MLP_INPUT_DATA_BITS * read_inputs_r) + MLP_INPUT_DATA_BITS - 1 downto (MLP_INPUT_DATA_BITS * read_inputs_r)) <= data_in(MLP_INPUT_DATA_BITS - 1 downto 0);
                    read_inputs <= read_inputs_r + 1;
                    if data_in_last = '1' then
                        ns          <= feed_mlp;
                        read_inputs <= 0;
                    end if;
                end if;

            when feed_mlp =>
                -- Feed the MLP with the input data
                activation_ap_valid <= '1';
                ap_start            <= '1';
                -- Move to wait_mlp_ready state
                ns                  <= wait_mlp_ready;

            when wait_mlp_ready =>
                if ap_ready_r = '1' then
                    -- Move to wait_mlp_done state
                    ns <= wait_mlp_done;
                else
                    -- Remains ap_start until ap_ready = '1'
                    ap_start <= '1';
                end if;

            when wait_mlp_done =>
                if ap_done = '1' then
                    -- Move to write_axi_outputs state
                    ns <= write_axi_outputs;
                end if;

            when write_axi_outputs =>
                if data_out_ready = '1' then
                    -- Write the output data
                    data_out(MLP_OUTPUT_BIT_WIDTH - 1 downto 0) <= prediction_data(MLP_OUTPUT_BIT_WIDTH - 1 downto 0); -- Adjusted for single neuron
                    write_outputs <= write_outputs_r + 1;
                    data_out_valid <= '1';
                    -- Determine if this is the last output
                    if write_outputs_r + 1 = MLP_OUTPUT_NEURONS then  -- Ensure MLP_OUTPUT_NEURONS is defined in mlp_top_pkg
                        ns            <= idle;
                        write_outputs <= 0;
                        data_out_last <= '1';
                    end if;
                end if;

        end case;
    end process b_conv;

    -- Instantiate the MLP Module
    inst_mlp : entity work.myproject  -- Replace 'myproject' with your actual MLP module name
        port map(
            ap_clk                  => clk,
            ap_rst                  => ap_rst,
            ap_start                => ap_start,
            ap_done                 => ap_done,
            ap_idle                 => ap_idle,
            ap_ready                => ap_ready,
            input_1_V               => activation_data,
            input_1_V_ap_vld        => activation_ap_valid,
            layer9_out_0_V          => prediction_data,
            layer9_out_0_V_ap_vld   => prediction_ap_valid  -- Not used in current design
        );

    -- Handle data_in_ready Signal
    data_in_ready <= ap_idle;

end architecture RTL;
