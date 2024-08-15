---------------------------------------------
--             MLP Testbench               --
---------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work; -- @suppress "Superfluous library clause: access to library 'work' is implicit"
use work.mlp_top_pkg.all;

entity mlp_1_neuron_tb is
    generic(
        CLK_PERIOD                : time    := 10 ns;
        AXI_DATA_WIDTH            : integer := 32
    );
end entity mlp_1_neuron_tb;

architecture RTL of mlp_1_neuron_tb is
    -- Signal declaration
    signal axi_clk       : std_logic;
    signal axi_rst_n     : std_logic;
    signal s_axis_tready : std_logic;
    signal s_axis_tdata  : std_logic_vector(AXI_DATA_WIDTH - 1 downto 0);
    signal s_axis_tlast  : std_logic;
    signal s_axis_tvalid : std_logic;
    signal m_axis_tvalid : std_logic; -- @suppress "signal m_axis_tvalid is never read"
    signal m_axis_tdata  : std_logic_vector(AXI_DATA_WIDTH - 1 downto 0); -- @suppress "signal m_axis_tdata is never read"
    signal m_axis_tlast  : std_logic;
    signal m_axis_tready : std_logic;
    signal num_inputs    : integer := 0;

begin

    -- Clock process generation
    CLK_PROCESS : process
    begin
        axi_clk <= '0';
        loop
            wait for CLK_PERIOD / 2;
            axi_clk <= not axi_clk;
        end loop;
    end process CLK_PROCESS;

    -- DUT instantiation
    DUT : entity work.mlp
        generic map(
            AXI_DATA_WIDTH       => AXI_DATA_WIDTH
        )
        port map(
            AXIS_ACLK     => axi_clk,
            AXI_ARESETN   => axi_rst_n,
            S_AXIS_TREADY => s_axis_tready,
            S_AXIS_TDATA  => s_axis_tdata,
            S_AXIS_TLAST  => s_axis_tlast,
            S_AXIS_TVALID => s_axis_tvalid,
            M_AXIS_TVALID => m_axis_tvalid,
            M_AXIS_TDATA  => m_axis_tdata,
            M_AXIS_TLAST  => m_axis_tlast,
            M_AXIS_TREADY => m_axis_tready
        );

    -- Stimuli process
    STIMULI_PROCESS : process
    begin
        -- Reset
        axi_rst_n     <= '0';
        m_axis_tready <= '1';
        s_axis_tdata  <= (others => '0');
        s_axis_tlast  <= '0';
        s_axis_tvalid <= '0';
        num_inputs   <= 0;
        wait for 100 ns;
        axi_rst_n     <= '1';
        wait until rising_edge(axi_clk);

        -- Write MM2S data
        while s_axis_tready = '1' and num_inputs < MLP_INPUT_NEURONS-1
        loop
            s_axis_tdata  <= std_logic_vector(to_unsigned(num_inputs, AXI_DATA_WIDTH));
            s_axis_tvalid <= '1';
            num_inputs  <= num_inputs + 1;
            wait until rising_edge(axi_clk);
        end loop;

        -- Write last MM2S data
        s_axis_tdata  <= std_logic_vector(to_unsigned(num_inputs, AXI_DATA_WIDTH));
        s_axis_tlast  <= '1';
        s_axis_tvalid <= '1';
        wait until rising_edge(axi_clk);
        
        s_axis_tlast  <= '0';
        s_axis_tvalid <= '0';

        -- Wait for S2MM data
        wait until m_axis_tlast = '1';

        wait until rising_edge(axi_clk);
        wait until rising_edge(axi_clk);
        wait until rising_edge(axi_clk);

        std.env.finish;
    end process STIMULI_PROCESS;
end architecture RTL;
