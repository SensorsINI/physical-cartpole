# Rebuild the full Zedboard design after HLS IP regeneration: refresh the IP
# catalog, upgrade stale IP instances inside the block design, then run
# synthesis + implementation to bitstream.
#
# Usage (from this directory):
#   vivado -mode batch -source refresh_ip_and_build.tcl

set upgrade_patterns {
    *median_filter*
    *secloc_shell*
    *secloc_gate*
    *nn_marshal*
}

proc upgrade_bd_ips {patterns} {
    set matched {}
    foreach pat $patterns {
        foreach ip [get_ips -quiet -filter "NAME =~ $pat"] {
            lappend matched $ip
        }
    }
    set matched [lsort -unique $matched]
    if {[llength $matched] > 0} {
        puts "Upgrading IP(s): $matched"
        if {[catch {upgrade_ip $matched} err]} {
            puts "upgrade_ip: $err (may already be current)"
        }
    }
}

open_project CartpoleDriverZynq_AXIS_Zedboard/CartpoleDriverZynq_AXIS_Zedboard.xpr

update_ip_catalog -rebuild

set bd_file [get_files -quiet *.bd]
if {$bd_file ne ""} {
    open_bd_design $bd_file
    upgrade_bd_ips $upgrade_patterns
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
