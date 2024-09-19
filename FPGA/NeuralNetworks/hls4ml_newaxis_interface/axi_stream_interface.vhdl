-- File: axi_stream_interface.vhdl

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity axi_stream_interface is
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
        M_AXIS_TREADY : in  std_logic;
        
        -- Interface to Network Control Module
        data_in          : out std_logic_vector(AXI_DATA_WIDTH-1 downto 0);
        data_in_valid    : out std_logic;
        data_in_last     : out std_logic;
        data_in_ready    : in  std_logic;
        
        data_out         : in  std_logic_vector(AXI_DATA_WIDTH-1 downto 0);
        data_out_valid   : in  std_logic;
        data_out_last    : in  std_logic;
        data_out_ready   : out std_logic
    );
end entity axi_stream_interface;

architecture RTL of axi_stream_interface is
    -- Internal Signals
begin

    -- AXI Stream Input Interface
    process(AXIS_ACLK, AXI_ARESETN)
    begin
        if AXI_ARESETN = '0' then
            data_in_valid  <= '0';
            data_in_last   <= '0';
        elsif rising_edge(AXIS_ACLK) then
            if S_AXIS_TVALID = '1' and data_in_ready = '1' then
                data_in       <= S_AXIS_TDATA;
                data_in_valid <= '1';
                data_in_last  <= S_AXIS_TLAST;
            else
                data_in_valid <= '0';
                data_in_last  <= '0';
            end if;
        end if;
    end process;

    -- AXI Stream Output Interface
    process(AXIS_ACLK, AXI_ARESETN)
    begin
        if AXI_ARESETN = '0' then
            M_AXIS_TVALID  <= '0';
            M_AXIS_TDATA   <= (others => '0');
            M_AXIS_TLAST   <= '0';
        elsif rising_edge(AXIS_ACLK) then
            if data_out_valid = '1' and M_AXIS_TREADY = '1' then
                M_AXIS_TDATA  <= data_out;
                M_AXIS_TVALID <= '1';
                M_AXIS_TLAST  <= data_out_last;
            else
                M_AXIS_TVALID <= '0';
                M_AXIS_TLAST  <= '0';
            end if;
        end if;
    end process;

    -- Corrected: Set S_AXIS_TREADY directly based on data_in_ready
    S_AXIS_TREADY  <= data_in_ready;

    -- data_out_ready is correctly driven by M_AXIS_TREADY
    data_out_ready <= M_AXIS_TREADY;

end architecture RTL;
