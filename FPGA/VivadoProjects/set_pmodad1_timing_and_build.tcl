# Set PmodAD1 SPI counts to 40/120/1000/800 on the live Zybo BD, then
# regenerate the module_ref wrapper and rebuild the bitstream.
#
# Verilog defaults alone are not enough: the generated wrapper/XCI latches
# AD1_CLOCKS_* and a stale 20 @ 100 MHz breaks AD7476 framing.
#
# Usage (from this directory):
#   /tools/Xilinx/Vivado/2020.1/bin/vivado -mode batch -source set_pmodad1_timing_and_build.tcl

set proj_rel "CartpoleDriverZynq_AXIS_secloc/CartpoleDriverZynq_AXIS_secloc.xpr"
set xsa_path "cartpole_zybo_slider_timing.xsa"
if { $::argc > 0 } {
    set proj_rel [lindex $::argv 0]
}
if { $::argc > 1 } {
    set xsa_path [lindex $::argv 1]
}

open_project $proj_rel
open_bd_design [get_files cartpole_driver_design.bd]

set ad1 [get_bd_cells -quiet PmodAD1]
if { $ad1 eq "" } {
    puts "ERROR: BD cell PmodAD1 not found"
    exit 1
}

set_property -dict [ list \
    CONFIG.AD1_CLOCKS_PER_BIT {40} \
    CONFIG.AD1_CLOCKS_BEFORE_DATA {120} \
    CONFIG.AD1_CLOCKS_AFTER_DATA {1000} \
    CONFIG.AD1_CLOCKS_BETWEEN_TRANSACTIONS {800} \
] $ad1

puts "PmodAD1 CONFIG.AD1_CLOCKS_PER_BIT=[get_property CONFIG.AD1_CLOCKS_PER_BIT $ad1]"
puts "PmodAD1 CONFIG.AD1_CLOCKS_BEFORE_DATA=[get_property CONFIG.AD1_CLOCKS_BEFORE_DATA $ad1]"
puts "PmodAD1 CONFIG.AD1_CLOCKS_AFTER_DATA=[get_property CONFIG.AD1_CLOCKS_AFTER_DATA $ad1]"
puts "PmodAD1 CONFIG.AD1_CLOCKS_BETWEEN_TRANSACTIONS=[get_property CONFIG.AD1_CLOCKS_BETWEEN_TRANSACTIONS $ad1]"

validate_bd_design
save_bd_design

set bd_file [get_files cartpole_driver_design.bd]
generate_target all $bd_file

set wrap [file join [file dirname $bd_file] ip cartpole_driver_design_PmodAD1_0 synth cartpole_driver_design_PmodAD1_0.v]
if { ![file exists $wrap] } {
    puts "ERROR: generated PmodAD1 wrapper missing: $wrap"
    exit 1
}
set fh [open $wrap r]
set wrap_txt [read $fh]
close $fh
if { ![regexp {AD1_CLOCKS_PER_BIT\(40\)} $wrap_txt] } {
    puts "ERROR: generated wrapper still does not instantiate AD1_CLOCKS_PER_BIT(40)"
    exit 1
}
if { [regexp {AD1_CLOCKS_PER_BIT\(20\)} $wrap_txt] } {
    puts "ERROR: generated wrapper still has AD1_CLOCKS_PER_BIT(20)"
    exit 1
}
puts "WRAPPER_OK $wrap"

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
