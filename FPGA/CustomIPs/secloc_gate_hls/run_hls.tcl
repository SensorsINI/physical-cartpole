# Build the SecLoc gate IP: C synthesis and IP export.
#
# Usage (from this directory):
#   vitis_hls -f run_hls.tcl

set CUSTOM_IPS_DIR ".."
set GATE_DIR "../../../Firmware/Src/General"
set CFLAGS "-I${CUSTOM_IPS_DIR} -I${GATE_DIR} -I/usr/include/x86_64-linux-gnu"

open_project secloc_gate_hls
set_top secloc_gate
add_files secloc_gate.cpp -cflags ${CFLAGS}

open_solution "solution1"
set_part {xc7z020clg484-1}
create_clock -period 10 -name default

csynth_design

if {[catch {export_design -format ip_catalog -ipname secloc_gate} export_err]} {
    puts "export_design failed ($export_err); applying core_revision overflow workaround"
    set ip_dir secloc_gate_hls/solution1/impl/ip
    set f [open $ip_dir/run_ippack.tcl r]
    set content [read $f]
    close $f
    regsub -line {^set Revision    "2[0-9]} $content {set Revision    "20} content
    set f [open $ip_dir/run_ippack.tcl w]
    puts -nonewline $f $content
    close $f
    exec sh -c "cd $ip_dir && ./pack.sh" >@ stdout
}

exit
