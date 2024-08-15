# Contraints - Pinout
# Comment out sections for the blocks you delete

# XADC; JA; Angle measurement
set_property IOSTANDARD LVCMOS33 [get_ports Vaux15_0_v_n]
set_property IOSTANDARD LVCMOS33 [get_ports Vaux15_0_v_p]
set_property PACKAGE_PIN J16 [get_ports Vaux15_0_v_n]


# Motor and Encoder; JE
set_property PACKAGE_PIN H15 [get_ports PWM]
set_property PACKAGE_PIN W16 [get_ports Direction_1]
set_property PACKAGE_PIN J15 [get_ports Direction_2]
set_property PACKAGE_PIN V12 [get_ports {Motor_STBY[0]}]

set_property IOSTANDARD LVCMOS33 [get_ports PWM]
set_property IOSTANDARD LVCMOS33 [get_ports Direction_1]
set_property IOSTANDARD LVCMOS33 [get_ports Direction_2]
set_property IOSTANDARD LVCMOS33 [get_ports {Motor_STBY[0]}]

set_property IOSTANDARD LVCMOS33 [get_ports Pmod_in_Sensefeedback_A]
set_property IOSTANDARD LVCMOS33 [get_ports Pmod_in_Sensefeedback_B]
set_property PACKAGE_PIN T17 [get_ports Pmod_in_Sensefeedback_A]
set_property PACKAGE_PIN Y17 [get_ports Pmod_in_Sensefeedback_B]


# UART; JD
set_property PACKAGE_PIN T14 [get_ports rtsn_0]
set_property PACKAGE_PIN T15 [get_ports tx]
set_property PACKAGE_PIN P14 [get_ports rx]
set_property PACKAGE_PIN R14 [get_ports ctsn_0]
set_property IOSTANDARD LVCMOS33 [get_ports ctsn_0]
set_property IOSTANDARD LVCMOS33 [get_ports rtsn_0]
set_property IOSTANDARD LVCMOS33 [get_ports rx]
set_property IOSTANDARD LVCMOS33 [get_ports tx]


# UART; JC
set_property IOSTANDARD LVCMOS33 [get_ports uart_rtl_rxd]
set_property IOSTANDARD LVCMOS33 [get_ports uart_rtl_txd]
set_property PACKAGE_PIN T11 [get_ports uart_rtl_rxd]
set_property PACKAGE_PIN W15 [get_ports uart_rtl_txd]
set_property DRIVE 12 [get_ports uart_rtl_txd]
set_property SLEW SLOW [get_ports uart_rtl_txd]

set_property PACKAGE_PIN V15 [get_ports {uartlite_rtsn[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {uartlite_rtsn[0]}]
set_property PULLDOWN true [get_ports {uartlite_rtsn[0]}]


# JB
set_property IOSTANDARD LVCMOS33 [get_ports {EQUILIBRIUM_SWITCH_tri_i[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {EQUILIBRIUM_SWITCH_tri_i[0]}]
set_property PACKAGE_PIN V8 [get_ports {EQUILIBRIUM_SWITCH_tri_i[1]}]
set_property PACKAGE_PIN W8 [get_ports {EQUILIBRIUM_SWITCH_tri_i[0]}]
set_property PULLDOWN true [get_ports {EQUILIBRIUM_SWITCH_tri_i[1]}]
set_property PULLDOWN true [get_ports {EQUILIBRIUM_SWITCH_tri_i[0]}]


## JB; 2 ADC, currently 1 used for slider
set_property PACKAGE_PIN Y7 [get_ports cs]
set_property PACKAGE_PIN W6 [get_ports sclk]
set_property PACKAGE_PIN Y6 [get_ports ad1_d0]
set_property PACKAGE_PIN V6 [get_ports ad1_d1]
set_property IOSTANDARD LVCMOS33 [get_ports ad1_d0]
set_property IOSTANDARD LVCMOS33 [get_ports ad1_d1]
set_property IOSTANDARD LVCMOS33 [get_ports cs]
set_property IOSTANDARD LVCMOS33 [get_ports sclk]

## Debug block
#set_property C_ENABLE_CLK_DIVIDER false [get_debug_cores dbg_hub]
#set_property C_USER_SCAN_CHAIN 1 [get_debug_cores dbg_hub]
#set_property C_CLK_INPUT_FREQ_HZ 300000000 [get_debug_cores dbg_hub]
#connect_debug_port dbg_hub/clk [get_nets clk]
