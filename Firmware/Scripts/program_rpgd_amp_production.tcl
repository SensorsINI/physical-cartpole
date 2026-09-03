# CartPoleFirmware dual-core RPGD: slider bitstream + CPU1 worker + CPU0 AMP ELF.
# JTAG only. Does not write QSPI.

set repo_root [file normalize [file join [file dirname [file normalize [info script]]] ../..]]
set bit [file join $repo_root FPGA bitstreams cartpole_short_pole_secloc.bit]
set ps7 [file join $repo_root Firmware VitisProjects cartpole_zybo_secloc2026 hw ps7_init.tcl]
set cpu0 [file join $repo_root Firmware AmpWorkspace RPGD_AMP CartPoleFirmware_rpgd_amp_cpu0.elf]
set cpu1 [file join $repo_root Firmware AmpWorkspace RPGD_AMP RPGDWorker_cpu1.elf]

puts "BIT $bit"
puts "PS7 $ps7"
puts "CPU0 $cpu0"
puts "CPU1 $cpu1"

foreach f [list $bit $ps7 $cpu0 $cpu1] {
    if {![file exists $f]} {
        puts "ERROR: missing $f"
        puts "Build first: Firmware/Scripts/build_rpgd_amp_production.sh"
        exit 1
    }
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
    targets -set -filter {name =~ "*Cortex-A9 MPCore #0*"}
}
catch {targets -set -filter {name =~ "*Cortex-A9 MPCore #0*"}}
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

puts "Loading CPU1 $cpu1"
if {[catch {targets -set -filter {name =~ "*Cortex-A9 MPCore #1*"}} t1err]} {
    puts "ERROR: CPU1 target missing: $t1err"
    exit 1
}
catch {rst -processor}
if {[catch {dow $cpu1} d1err]} {
    puts "DOW_CPU1_FAIL $d1err"
    exit 1
}
puts "DOW_CPU1_OK"

puts "Initializing AMP mailbox at 0xFFFF0000 before releasing CPU1"
targets -set -filter {name =~ "*Cortex-A9 MPCore #0*"}
set mb_addr 0xFFFF0000
# magic, abi, STARTING, then the rest of the 104-byte mailbox
set mb_words [list 0x52504744 1 1]
for {set i 0} {$i < 23} {incr i} { lappend mb_words 0 }
foreach w $mb_words {
    mwr $mb_addr $w
    incr mb_addr 4
}
puts "mailbox magic=[mrd -value 0xFFFF0000] abi=[mrd -value 0xFFFF0004] state=[mrd -value 0xFFFF0008]"

targets -set -filter {name =~ "*Cortex-A9 MPCore #1*"}
con

puts "Waiting for CPU1 READY (mailbox+8 worker_state == 2)..."
set ready 0
for {set i 0} {$i < 50} {incr i} {
    after 100
    set st [mrd -value 0xFFFF0008]
    puts "worker_state=$st"
    if {$st eq "0x00000002" || $st eq "0x2" || $st == 2} {
        set ready 1
        break
    }
}
if {!$ready} {
    puts "ERROR: CPU1 did not reach READY. Leaving motor stopped."
    exit 1
}

puts "Loading CPU0 $cpu0"
targets -set -filter {name =~ "*Cortex-A9 MPCore #0*"}
if {[catch {dow $cpu0} d0err]} {
    puts "DOW_CPU0_FAIL $d0err"
    exit 1
}
puts "DOW_CPU0_OK"
con
after 500
puts "PROGRAM_OK rpgd_amp_production"
exit 0
