-- ==============================================================
-- Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2020.1 (64-bit)
-- Copyright 1986-2020 Xilinx, Inc. All Rights Reserved.
-- ==============================================================
library ieee; 
use ieee.std_logic_1164.all; 
use ieee.std_logic_unsigned.all;

entity nn_marshal_nn_marshal_handle_transaction_mask_table_rom is 
    generic(
             DWIDTH     : integer := 23; 
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


architecture rtl of nn_marshal_nn_marshal_handle_transaction_mask_table_rom is 

signal addr0_tmp : std_logic_vector(AWIDTH-1 downto 0); 
type mem_array is array (0 to MEM_SIZE-1) of std_logic_vector (DWIDTH-1 downto 0); 
signal mem : mem_array := (
    0 => "01111111111111111111111", 1 => "00111111111111111111111", 
    2 => "00011111111111111111111", 3 => "00001111111111111111111", 
    4 => "00000111111111111111111", 5 => "00000011111111111111111", 
    6 => "00000001111111111111111", 7 => "00000000111111111111111", 
    8 => "00000000011111111111111", 9 => "00000000001111111111111", 
    10 => "00000000000111111111111", 11 => "00000000000011111111111", 
    12 => "00000000000001111111111", 13 => "00000000000000111111111", 
    14 => "00000000000000011111111", 15 => "00000000000000001111111", 
    16 => "00000000000000000111111", 17 => "00000000000000000011111", 
    18 => "00000000000000000001111", 19 => "00000000000000000000111", 
    20 => "00000000000000000000011", 21 => "00000000000000000000001", 
    22 to 29=> "00000000000000000000000", 30 to 31=> "11111111111111111111111" );

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

entity nn_marshal_nn_marshal_handle_transaction_mask_table is
    generic (
        DataWidth : INTEGER := 23;
        AddressRange : INTEGER := 32;
        AddressWidth : INTEGER := 5);
    port (
        reset : IN STD_LOGIC;
        clk : IN STD_LOGIC;
        address0 : IN STD_LOGIC_VECTOR(AddressWidth - 1 DOWNTO 0);
        ce0 : IN STD_LOGIC;
        q0 : OUT STD_LOGIC_VECTOR(DataWidth - 1 DOWNTO 0));
end entity;

architecture arch of nn_marshal_nn_marshal_handle_transaction_mask_table is
    component nn_marshal_nn_marshal_handle_transaction_mask_table_rom is
        port (
            clk : IN STD_LOGIC;
            addr0 : IN STD_LOGIC_VECTOR;
            ce0 : IN STD_LOGIC;
            q0 : OUT STD_LOGIC_VECTOR);
    end component;



begin
    nn_marshal_nn_marshal_handle_transaction_mask_table_rom_U :  component nn_marshal_nn_marshal_handle_transaction_mask_table_rom
    port map (
        clk => clk,
        addr0 => address0,
        ce0 => ce0,
        q0 => q0);

end architecture;


