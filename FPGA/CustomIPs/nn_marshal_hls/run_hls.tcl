# Build the per-network marshal IP: C synthesis and IP export.
#
# Usage (from this directory):
#   vitis_hls -f run_hls.tcl
#
# Re-run whenever nn_marshal_config.h changes in the deployed network folder.

set CUSTOM_IPS_DIR ".."
if {[info exists ::env(SECLOC_NN_NETWORK_DIR)] && $::env(SECLOC_NN_NETWORK_DIR) ne ""} {
    set NORM_DIR $::env(SECLOC_NN_NETWORK_DIR)
} else {
    set NORM_DIR "../../NeuralNetworks/hls4ml_dense_1out_8_07_07_2026"
}
puts "nn_marshal network include: ${NORM_DIR}"
set CFLAGS "-I${CUSTOM_IPS_DIR} -I${NORM_DIR} -I/usr/include/x86_64-linux-gnu"

open_project nn_marshal_hls
set_top nn_marshal
add_files nn_marshal.cpp -cflags ${CFLAGS}
add_files -tb nn_marshal_tb.cpp -cflags ${CFLAGS}

open_solution "solution1"
set_part {xc7z020clg484-1}
create_clock -period 10 -name default

csynth_design

if {[catch {export_design -format ip_catalog -ipname nn_marshal} export_err]} {
    puts "export_design failed ($export_err); applying core_revision overflow workaround"
    set ip_dir nn_marshal_hls/solution1/impl/ip
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
