# Rebuild bitstream after a network swap that changed nn_marshal_config.h.
#
# Prefer the shell wrapper (runs nn_marshal HLS first):
#   FPGA/VivadoProjects/swap_nn_and_build.sh [network_folder]
#
# Or, if marshal HLS was already re-run:
#   vivado -mode batch -source swap_nn_and_build.tcl

open_project CartpoleDriverZynq_AXIS_Zedboard/CartpoleDriverZynq_AXIS_Zedboard.xpr

update_ip_catalog -rebuild

set bd_file [get_files -quiet *.bd]
if {$bd_file ne ""} {
    open_bd_design $bd_file
    set marshal_ips [get_ips -quiet -filter {NAME =~ *nn_marshal*}]
    if {[llength $marshal_ips] > 0} {
        puts "Upgrading marshal IP(s): $marshal_ips"
        if {[catch {upgrade_ip $marshal_ips} err]} {
            puts "upgrade_ip: $err (may already be current)"
        }
    } else {
        puts "WARNING: no nn_marshal IP in block design"
    }
    save_bd_design
    catch {generate_target all $bd_file}
}

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

puts "BITSTREAM OK"
exit
