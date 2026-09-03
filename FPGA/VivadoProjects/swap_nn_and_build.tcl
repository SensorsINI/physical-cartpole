# Rebuild bitstream after a network swap that changed nn_marshal_config.h.
#
# Prefer the shell wrapper (runs nn_marshal HLS first):
#   FPGA/VivadoProjects/swap_nn_and_build.sh [network_folder]
#
# Or, if marshal HLS was already re-run:
#   vivado -mode batch -source swap_nn_and_build.tcl

open_project CartpoleDriverZynq_AXIS_secloc/CartpoleDriverZynq_AXIS_secloc.xpr

update_ip_catalog -rebuild

set bd_file [get_files -quiet *.bd]
if {$bd_file ne ""} {
    open_bd_design $bd_file
    # The generated BD IP has this stable name. Broad NAME/VLNV filters
    # silently missed it in Vivado 2020.1 and left the old checkpoint locked.
    set marshal_ips [get_ips -quiet cartpole_driver_design_NN_MARSHAL_0_0]
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

# CONTROLLER_AXIS is synthesized out of context together with the adapter and
# myproject hierarchy. Vivado 2020.1 does not reliably invalidate that run when
# only external VHDL sources are swapped; explicitly rebuilding it prevents a
# previous network (for example Dense-8/12-bit) from surviving in a new BIT.
set controller_ooc_run [get_runs -quiet cartpole_driver_design_CONTROLLER_AXIS_0_0_synth_1]
if {[llength $controller_ooc_run] != 1} {
    puts "ERROR: expected one CONTROLLER_AXIS out-of-context synthesis run"
    exit 1
}
reset_run $controller_ooc_run
launch_runs $controller_ooc_run -jobs 8
wait_on_run $controller_ooc_run
set controller_progress [get_property PROGRESS $controller_ooc_run]
set controller_status [get_property STATUS $controller_ooc_run]
puts "controller OOC progress: $controller_progress"
puts "controller OOC status: $controller_status"
if {$controller_progress ne "100%"} {
    puts "CONTROLLER OOC BUILD FAILED"
    exit 1
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

# Refuse a successful-looking BIT if the controller OOC checkpoint still
# contains the Dense-8 12-bit network instead of the IROS 14-bit hierarchy.
open_run impl_1
set short_pole_cells [get_cells -hier -quiet -filter {
    REF_NAME =~ "*tanh_ap_fixed_22_8*ap_fixed_14_2*"
}]
set dense8_cells [get_cells -hier -quiet -filter {
    REF_NAME =~ "*tanh_ap_fixed_18_6*ap_fixed_12_1*"
}]
puts "short-pole signature cells: [llength $short_pole_cells]"
puts "Dense-8 signature cells: [llength $dense8_cells]"
if {[llength $short_pole_cells] == 0 || [llength $dense8_cells] != 0} {
    puts "NETWORK IDENTITY CHECK FAILED"
    exit 1
}

puts "BITSTREAM OK"
exit
