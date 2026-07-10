# Build bitstream and export the hardware platform (XSA, bitstream included).
# Does NOT touch the Vitis workspace / BSP - linking the new platform into
# firmware is a separate, manual step.
#
# Usage (from this directory):
#   vivado -mode batch -source build_and_export_xsa.tcl [-tclargs <xsa_path>]

set xsa_path "cartpole_zedboard_secloc_three_block.xsa"
if { $::argc > 0 } {
    set xsa_path [lindex $::argv 0]
}

open_project CartpoleDriverZynq_AXIS_Zedboard/CartpoleDriverZynq_AXIS_Zedboard.xpr

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
exit
