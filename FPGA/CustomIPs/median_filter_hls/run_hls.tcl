# Regenerate the XADC filter IP: C simulation, C synthesis, and IP export.
#
# Usage (from this directory):
#   vitis_hls -f run_hls.tcl
#
# Output IP lands in median_filter_hls/solution1/impl/ip (the location the
# Vivado block-design scripts already reference as xilinx.com:hls:median_filter:1.0).
#
# Clock: the block is driven by FCLK_CLK1 = 100 MHz in the Zynq block design.
# Part:  Zybo Z7-20 (xc7z020clg400-1). For the Zedboard use xc7z020clg484-1;
#        the RTL is identical for both, so one export serves both boards.

open_project median_filter_hls
set_top median_filter
add_files median_filter.cpp
add_files median_functions.cpp
add_files -tb median_filter_tb.cpp

open_solution "solution1"
set_part {xc7z020clg400-1}
create_clock -period 10 -name default

# csim_design is skipped: Vitis HLS 2020.1 ships a binutils too old to link
# against modern glibc (fails on '.relr.dyn'). Run the same testbench with the
# system compiler instead:
#   g++ -std=c++14 -I$XILINX_VITIS/include -o median_tb \
#       median_filter_tb.cpp median_filter.cpp median_functions.cpp && ./median_tb
# csim_design

csynth_design

# export_design hits the Xilinx "Y2K22" overflow: core_revision is derived from
# the current date (YYMMDDHHMM) and no longer fits in a 32-bit int (since 2022,
# and again for 2026+ with the official patch level shipped in 2020.1).
# Workaround: rewrite the revision in the generated packaging script and re-run it.
if {[catch {export_design -format ip_catalog} export_err]} {
    puts "export_design failed ($export_err); applying core_revision overflow workaround"
    set ip_dir median_filter_hls/solution1/impl/ip
    set f [open $ip_dir/run_ippack.tcl r]
    set content [read $f]
    close $f
    # Cap the revision below 2^31 by replacing the two leading year digits with 20.
    regsub -line {^set Revision    "2[0-9]} $content {set Revision    "20} content
    set f [open $ip_dir/run_ippack.tcl w]
    puts -nonewline $f $content
    close $f
    exec sh -c "cd $ip_dir && ./pack.sh" >@ stdout
}

exit
