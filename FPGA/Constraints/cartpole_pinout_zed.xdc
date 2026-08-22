# Contraints - Pinout for Zedboard platform
# Comment out sections for the blocks you delete

# XADC; XADC HEADER; Angle measurement
set_property IOSTANDARD LVCMOS33 [get_ports Vaux0_0_v_n]
set_property IOSTANDARD LVCMOS33 [get_ports Vaux0_0_v_p]
set_property PACKAGE_PIN E16 [get_ports Vaux0_0_v_n]


# Motor and Encoder; JA
set_property IOSTANDARD LVCMOS33 [get_ports PWM]
set_property IOSTANDARD LVCMOS33 [get_ports Direction_1]
set_property IOSTANDARD LVCMOS33 [get_ports Direction_2]
set_property IOSTANDARD LVCMOS33 [get_ports {Motor_STBY[0]}]
set_property PACKAGE_PIN W11 [get_ports PWM]
set_property PACKAGE_PIN W8 [get_ports Direction_1]
set_property PACKAGE_PIN V10 [get_ports Direction_2]
set_property PACKAGE_PIN AB6 [get_ports {Motor_STBY[0]}]

set_property IOSTANDARD LVCMOS33 [get_ports Pmod_in_Sensefeedback_A]
set_property IOSTANDARD LVCMOS33 [get_ports Pmod_in_Sensefeedback_B]
set_property PACKAGE_PIN V8 [get_ports Pmod_in_Sensefeedback_A]
set_property PACKAGE_PIN V9 [get_ports Pmod_in_Sensefeedback_B]

# UART; JD
set_property IOSTANDARD LVCMOS33 [get_ports ctsn_0]
set_property IOSTANDARD LVCMOS33 [get_ports rtsn_0]
set_property IOSTANDARD LVCMOS33 [get_ports rx]
set_property IOSTANDARD LVCMOS33 [get_ports tx]
set_property PACKAGE_PIN AB7 [get_ports rtsn_0]
set_property PACKAGE_PIN AA4 [get_ports tx]
set_property PACKAGE_PIN Y4 [get_ports rx]
set_property PACKAGE_PIN T6 [get_ports ctsn_0]


# UART; JC
set_property IOSTANDARD LVCMOS33 [get_ports uart_rtl_rxd]
set_property IOSTANDARD LVCMOS33 [get_ports uart_rtl_txd]
set_property PACKAGE_PIN R6 [get_ports uart_rtl_rxd]
set_property PACKAGE_PIN U4 [get_ports uart_rtl_txd]
set_property DRIVE 12 [get_ports uart_rtl_txd]
set_property SLEW SLOW [get_ports uart_rtl_txd]

set_property PACKAGE_PIN T4 [get_ports {uartlite_rtsn[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {uartlite_rtsn[0]}]
set_property PULLDOWN true [get_ports {uartlite_rtsn[0]}]


# JB
set_property IOSTANDARD LVCMOS33 [get_ports {EQUILIBRIUM_SWITCH_tri_i[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {EQUILIBRIUM_SWITCH_tri_i[0]}]
set_property PACKAGE_PIN W7 [get_ports {EQUILIBRIUM_SWITCH_tri_i[1]}]
set_property PACKAGE_PIN V7 [get_ports {EQUILIBRIUM_SWITCH_tri_i[0]}]
set_property PULLDOWN true [get_ports {EQUILIBRIUM_SWITCH_tri_i[1]}]
set_property PULLDOWN true [get_ports {EQUILIBRIUM_SWITCH_tri_i[0]}]


# JB; 2 ADC, currently 1 used for slider
set_property PACKAGE_PIN V4 [get_ports cs]
set_property PACKAGE_PIN V5 [get_ports sclk]
set_property PACKAGE_PIN W5 [get_ports ad1_d0]
set_property PACKAGE_PIN W6 [get_ports ad1_d1]
set_property IOSTANDARD LVCMOS33 [get_ports ad1_d0]
set_property IOSTANDARD LVCMOS33 [get_ports ad1_d1]
set_property IOSTANDARD LVCMOS33 [get_ports cs]
set_property IOSTANDARD LVCMOS33 [get_ports sclk]

# Set sws_8bits to 3.3V to avoid conflicts with Vaux0_0_v_n
set_property IOSTANDARD LVCMOS33 [get_ports {sws_8bits_tri_i[7]}]
set_property IOSTANDARD LVCMOS33 [get_ports {sws_8bits_tri_i[6]}]
set_property IOSTANDARD LVCMOS33 [get_ports {sws_8bits_tri_i[5]}]
set_property IOSTANDARD LVCMOS33 [get_ports {sws_8bits_tri_i[4]}]
set_property IOSTANDARD LVCMOS33 [get_ports {sws_8bits_tri_i[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {sws_8bits_tri_i[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {sws_8bits_tri_i[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {sws_8bits_tri_i[0]}]
