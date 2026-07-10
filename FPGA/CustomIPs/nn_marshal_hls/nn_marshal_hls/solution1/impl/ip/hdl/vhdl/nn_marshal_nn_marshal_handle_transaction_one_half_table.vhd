-- ==============================================================
-- Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2020.1 (64-bit)
-- Copyright 1986-2020 Xilinx, Inc. All Rights Reserved.
-- ==============================================================
library ieee; 
use ieee.std_logic_1164.all; 
use ieee.std_logic_unsigned.all;

entity nn_marshal_nn_marshal_handle_transaction_one_half_table_rom is 
    generic(
             DWIDTH     : integer := 24; 
             AWIDTH     : integer := 5; 
             MEM_SIZE    : integer := 32
    ); 
    port (
          addr0      : in std_logic_vector(AWIDTH-1 downto 0); 
          ce0       : in std_logic; 
          q0         : out std_logic_vector(DWIDTH-1 downto 0);
          clk       : in std_logic
    ); 
end entity; 


architecture rtl of nn_marshal_nn_marshal_handle_transaction_one_half_table_rom is 

signal addr0_tmp : std_logic_vector(AWIDTH-1 downto 0); 
type mem_array is array (0 to MEM_SIZE-1) of std_logic_vector (DWIDTH-1 downto 0); 
signal mem : mem_array := (
    0 => "001000000000000000000000", 1 => "000100000000000000000000", 
    2 => "000010000000000000000000", 3 => "000001000000000000000000", 
    4 => "000000100000000000000000", 5 => "000000010000000000000000", 
    6 => "000000001000000000000000", 7 => "000000000100000000000000", 
    8 => "000000000010000000000000", 9 => "000000000001000000000000", 
    10 => "000000000000100000000000", 11 => "000000000000010000000000", 
    12 => "000000000000001000000000", 13 => "000000000000000100000000", 
    14 => "000000000000000010000000", 15 => "000000000000000001000000", 
    16 => "000000000000000000100000", 17 => "000000000000000000010000", 
    18 => "000000000000000000001000", 19 => "000000000000000000000100", 
    20 => "000000000000000000000010", 21 => "000000000000000000000001", 
    22 to 29=> "000000000000000000000000", 30 => "100000000000000000000000", 
    31 => "010000000000000000000000" );

attribute syn_rom_style : string;
attribute syn_rom_style of mem : signal is "select_rom";
attribute ROM_STYLE : string;
attribute ROM_STYLE of mem : signal is "distributed";

begin 


memory_access_guard_0: process (addr0) 
begin
      addr0_tmp <= addr0;
--synthesis translate_off
      if (CONV_INTEGER(addr0) > mem_size-1) then
           addr0_tmp <= (others => '0');
      else 
           addr0_tmp <= addr0;
      end if;
--synthesis translate_on
end process;

p_rom_access: process (clk)  
begin 
    if (clk'event and clk = '1') then
        if (ce0 = '1') then 
            q0 <= mem(CONV_INTEGER(addr0_tmp)); 
        end if;
    end if;
end process;

end rtl;

Library IEEE;
use IEEE.std_logic_1164.all;

entity nn_marshal_nn_marshal_handle_transaction_one_half_table is
    generic (
        DataWidth : INTEGER := 24;
        AddressRange : INTEGER := 32;
        AddressWidth : INTEGER := 5);
    port (
        reset : IN STD_LOGIC;
        clk : IN STD_LOGIC;
        address0 : IN STD_LOGIC_VECTOR(AddressWidth - 1 DOWNTO 0);
        ce0 : IN STD_LOGIC;
        q0 : OUT STD_LOGIC_VECTOR(DataWidth - 1 DOWNTO 0));
end entity;

architecture arch of nn_marshal_nn_marshal_handle_transaction_one_half_table is
    component nn_marshal_nn_marshal_handle_transaction_one_half_table_rom is
        port (
            clk : IN STD_LOGIC;
            addr0 : IN STD_LOGIC_VECTOR;
            ce0 : IN STD_LOGIC;
            q0 : OUT STD_LOGIC_VECTOR);
    end component;



begin
    nn_marshal_nn_marshal_handle_transaction_one_half_table_rom_U :  component nn_marshal_nn_marshal_handle_transaction_one_half_table_rom
    port map (
        clk => clk,
        addr0 => address0,
        ce0 => ce0,
        q0 => q0);

end architecture;


