# CartPoleFirmware: slider-timing bitstream (AD1 40/120/1000/800) + ELF.

set bit /home/marcin/PycharmProjects/physical-cartpole/FPGA/VivadoProjects/CartpoleDriverZynq_AXIS_secloc/CartpoleDriverZynq_AXIS_secloc.runs/impl_1/cartpole_driver_design_wrapper.bit
set elf /home/marcin/PycharmProjects/physical-cartpole/Firmware/VitisProjects/CartPoleFirmware/Debug/CartPoleFirmware.elf
set ps7 /home/marcin/PycharmProjects/physical-cartpole/Firmware/VitisProjects/cartpole_zybo_secloc2026/hw/ps7_init.tcl

puts "BIT $bit"
puts "ELF $elf"
puts "PS7 $ps7"

if {![file exists $bit] || ![file exists $elf] || ![file exists $ps7]} {
    puts "ERROR: bit/elf/ps7 missing"
    exit 1
}

if {[catch {connect -url TCP:127.0.0.1:3121} cerr]} {
    puts "connect url failed ($cerr), trying connect"
    connect
}
after 3000

set opened 0
for {set i 1} {$i <= 12} {incr i} {
    puts "==== jtag scan $i ===="
    set jt ""
    if {[catch {set jt [jtag targets]} e]} {
        puts "jtag targets err: $e"
        after 1000
        continue
    }
    puts $jt
    if {[regexp {^\s*(\d+)\s+} $jt -> jid]} {
        puts "Opening jtag id $jid"
        if {[catch {jtag targets $jid} oe]} {
            puts "WARN open $jid: $oe"
        } else {
            set opened 1
            after 1500
            break
        }
    }
    after 1000
}

if {!$opened} {
    puts "ERROR: Digilent cable never appeared. Close Vitis first."
    exit 1
}

if {[catch {targets -set -nocase -filter {name =~ "APU*"}}]} {
    targets -set -filter {name =~ "ARM Cortex-A9 MPCore #0"}
}
catch {targets -set -filter {name =~ "ARM Cortex-A9 MPCore #0"}}
catch {stop}
catch {rst -system}
after 2000

puts "Programming FPGA..."
if {[catch {fpga $bit} ferr]} {
    puts "FPGA_FAIL $ferr"
    exit 1
}
puts "FPGA_OK"
after 1000
source $ps7
ps7_init
ps7_post_config
puts "PS7_OK"
catch {targets -set -filter {name =~ "ARM Cortex-A9 MPCore #0"}}
if {[catch {dow $elf} derr]} {
    puts "DOW_FAIL $derr"
    exit 1
}
puts "DOW_OK"
con
after 500
puts "PROGRAM_OK cartpole_slider"
exit 0
