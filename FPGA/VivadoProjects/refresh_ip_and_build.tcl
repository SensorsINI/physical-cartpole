# Rebuild the full Zedboard design after the median_filter HLS IP was
# regenerated: refresh the IP catalog, upgrade the IP instance inside the block
# design (Vivado otherwise keeps using the cached/locked old revision), then
# run synthesis + implementation to bitstream.
#
# Usage (from this directory):
#   vivado -mode batch -source refresh_ip_and_build.tcl

open_project CartpoleDriverZynq_AXIS_Zedboard/CartpoleDriverZynq_AXIS_Zedboard.xpr

update_ip_catalog -rebuild

set bd_file [get_files -quiet *.bd]
if {$bd_file ne ""} {
    open_bd_design $bd_file
    set stale_ips [get_ips -quiet -filter {NAME =~ "*median_filter*"}]
    if {[llength $stale_ips] > 0} {
        puts "Upgrading IP(s): $stale_ips"
        if {[catch {upgrade_ip $stale_ips} err]} {
            puts "upgrade_ip: $err (may already be current)"
        }
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
