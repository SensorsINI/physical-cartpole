# Add Zybo BTN0-BTN3 as PL_BUTTONS_GPIO on the live Zybo BD, then bitstream + XSA.
#
# Usage (from this directory):
#   /tools/Xilinx/Vivado/2020.1/bin/vivado -mode batch -source add_pl_buttons_and_build.tcl
#
# Default project is CartpoleDriverZynq_AXIS_secloc (the current Zybo build).

set proj_rel "CartpoleDriverZynq_AXIS_secloc/CartpoleDriverZynq_AXIS_secloc.xpr"
set xsa_path "cartpole_zybo_pl_buttons.xsa"
if { $::argc > 0 } {
    set proj_rel [lindex $::argv 0]
}
if { $::argc > 1 } {
    set xsa_path [lindex $::argv 1]
}

open_project $proj_rel
open_bd_design [get_files cartpole_driver_design.bd]

if { [get_bd_cells -quiet PL_BUTTONS_GPIO] eq "" } {
    puts "Adding PL_BUTTONS_GPIO (BTN0-BTN3)..."

    set PL_BUTTONS_GPIO [ create_bd_cell -type ip -vlnv xilinx.com:ip:axi_gpio:2.0 PL_BUTTONS_GPIO ]
    set_property -dict [ list \
        CONFIG.C_ALL_INPUTS {1} \
        CONFIG.C_GPIO_WIDTH {4} \
        CONFIG.C_INTERRUPT_PRESENT {1} \
        CONFIG.GPIO_BOARD_INTERFACE {Custom} \
        CONFIG.USE_BOARD_FLOW {true} \
    ] $PL_BUTTONS_GPIO

    if { [get_bd_intf_ports -quiet btns_4bits] eq "" } {
        create_bd_intf_port -mode Master -vlnv xilinx.com:interface:gpio_rtl:1.0 btns_4bits
    }
    connect_bd_intf_net [get_bd_intf_pins PL_BUTTONS_GPIO/GPIO] [get_bd_intf_ports btns_4bits]

    set ic [get_bd_cells axi_interconnect_0]
    set nmi [get_property CONFIG.NUM_MI $ic]
    if { $nmi < 11 } {
        set_property CONFIG.NUM_MI {11} $ic
    }
    connect_bd_intf_net [get_bd_intf_pins axi_interconnect_0/M10_AXI] [get_bd_intf_pins PL_BUTTONS_GPIO/S_AXI]
    connect_bd_net [get_bd_pins axi_interconnect_0/M10_ACLK] [get_bd_pins processing_system7_0/FCLK_CLK1]
    connect_bd_net [get_bd_pins axi_interconnect_0/M10_ARESETN] [get_bd_pins rst_ps7_0_100M/peripheral_aresetn]
    connect_bd_net [get_bd_pins PL_BUTTONS_GPIO/s_axi_aclk] [get_bd_pins processing_system7_0/FCLK_CLK1]
    connect_bd_net [get_bd_pins PL_BUTTONS_GPIO/s_axi_aresetn] [get_bd_pins rst_ps7_0_100M/peripheral_aresetn]

    set concat [get_bd_cells IRQ_CONCAT]
    set nports [get_property CONFIG.NUM_PORTS $concat]
    if { $nports < 4 } {
        set_property CONFIG.NUM_PORTS {4} $concat
    }
    connect_bd_net [get_bd_pins PL_BUTTONS_GPIO/ip2intc_irpt] [get_bd_pins IRQ_CONCAT/In3]

    assign_bd_address -offset 0x81230000 -range 0x00010000 \
        -target_address_space [get_bd_addr_spaces processing_system7_0/Data] \
        [get_bd_addr_segs PL_BUTTONS_GPIO/S_AXI/Reg] -force

    validate_bd_design
    save_bd_design
} else {
    puts "PL_BUTTONS_GPIO already present; skipping BD edits."
}

set bd_file [get_files cartpole_driver_design.bd]
generate_target all $bd_file

reset_run synth_1
launch_runs impl_1 -to_step write_bitstream -jobs 8
wait_on_run impl_1

set progress [get_property PROGRESS [get_runs impl_1]]
set status [get_property STATUS [get_runs impl_1]]
puts "impl_1 progress: $progress"
puts "impl_1 status: $status"
if {$progress ne "100%"} {
    puts "BUILD FAILED"
    exit 1
}

write_hw_platform -fixed -include_bit -force -file $xsa_path
puts "XSA WRITTEN: [file normalize $xsa_path]"
puts "BITSTREAM OK"
exit 0
