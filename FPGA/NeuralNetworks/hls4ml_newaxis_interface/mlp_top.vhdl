-- File: mlp_top.vhdl

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.mlp_top_pkg.all;

entity mlp_top is
    generic(
        AXI_DATA_WIDTH : integer := 32
    );
    port(
        -- AXI Stream Clock and Reset
        AXIS_ACLK     : in  std_logic;
        AXI_ARESETN   : in  std_logic;
        
        -- AXI Stream Input (from Zynq PS to FPGA)
        S_AXIS_TREADY : out std_logic;
        S_AXIS_TDATA  : in  std_logic_vector(AXI_DATA_WIDTH - 1 downto 0);
        S_AXIS_TLAST  : in  std_logic;
        S_AXIS_TVALID : in  std_logic;
        
        -- AXI Stream Output (from FPGA to Zynq PS)
        M_AXIS_TVALID : out std_logic;
        M_AXIS_TDATA  : out std_logic_vector(AXI_DATA_WIDTH - 1 downto 0);
        M_AXIS_TLAST  : out std_logic;
        M_AXIS_TREADY : in  std_logic
    );
end entity mlp_top;

architecture RTL of mlp_top is
    -- Signals between AXI Stream Interface and Network Control
    signal data_in          : std_logic_vector(AXI_DATA_WIDTH-1 downto 0);
    signal data_in_valid    : std_logic;
    signal data_in_last     : std_logic;
    signal data_in_ready    : std_logic;
    
    signal data_out         : std_logic_vector(AXI_DATA_WIDTH-1 downto 0);
    signal data_out_valid   : std_logic;
    signal data_out_last    : std_logic;
    signal data_out_ready   : std_logic;

begin

    -- Instantiate AXI Stream Interface
    axi_interface_inst : entity work.axi_stream_interface
        generic map(
            AXI_DATA_WIDTH => AXI_DATA_WIDTH
        )
        port map(
            AXIS_ACLK     => AXIS_ACLK,
            AXI_ARESETN   => AXI_ARESETN,
            S_AXIS_TREADY => S_AXIS_TREADY,
            S_AXIS_TDATA  => S_AXIS_TDATA,
            S_AXIS_TLAST  => S_AXIS_TLAST,
            S_AXIS_TVALID => S_AXIS_TVALID,
            M_AXIS_TVALID => M_AXIS_TVALID,
            M_AXIS_TDATA  => M_AXIS_TDATA,
            M_AXIS_TLAST  => M_AXIS_TLAST,
            M_AXIS_TREADY => M_AXIS_TREADY,
            data_in        => data_in,
            data_in_valid  => data_in_valid,
            data_in_last   => data_in_last,
            data_in_ready  => data_in_ready,
            data_out       => data_out,
            data_out_valid => data_out_valid,
            data_out_last  => data_out_last,
            data_out_ready => data_out_ready
        );

    -- Instantiate Network Control Module
    mlp_control_inst : entity work.mlp_control
        generic map(
            AXI_DATA_WIDTH => AXI_DATA_WIDTH
        )
        port map(
            clk            => AXIS_ACLK,
            reset_n        => AXI_ARESETN,
            data_in        => data_in,
            data_in_valid  => data_in_valid,
            data_in_last   => data_in_last,
            data_in_ready  => data_in_ready,
            data_out       => data_out,
            data_out_valid => data_out_valid,
            data_out_last  => data_out_last,
            data_out_ready => data_out_ready
        );

end architecture RTL;
