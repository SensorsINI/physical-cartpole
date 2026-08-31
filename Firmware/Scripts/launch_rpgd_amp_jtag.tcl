# Motor-safe JTAG launch for dual-core RPGD AMP.
# Programs the PL (clears motor registers), loads CPU1, waits for READY,
# then loads the CPU0 harness. Never flashes QSPI and never enables the motor.
#
# Usage:
#   xsct Firmware/Scripts/launch_rpgd_amp_jtag.tcl
# Optional:
#   -cpu0 ELF -cpu1 ELF -bit BIT -uart /dev/ttyUSB1

set script_dir [file dirname [file normalize [info script]]]
set repo_root [file normalize [file join $script_dir ../..]]
set cpu0 [file join $repo_root Firmware AmpWorkspace RPGD_AMP CartPoleFirmware_rpgd_amp_cpu0.elf]
set cpu1 [file join $repo_root Firmware AmpWorkspace RPGD_AMP RPGDWorker_cpu1.elf]
set bit  [file join $repo_root Firmware VitisProjects CartPoleFirmware _ide bitstream cartpole_zybo_secloc2026.bit]
set psinit [file join $repo_root Firmware VitisProjects CartPoleFirmware _ide psinit ps7_init.tcl]
set uart ""

for {set i 0} {$i < $argc} {incr i} {
	set a [lindex $argv $i]
	switch -- $a {
		-cpu0 { incr i; set cpu0 [lindex $argv $i] }
		-cpu1 { incr i; set cpu1 [lindex $argv $i] }
		-bit { incr i; set bit [lindex $argv $i] }
		-psinit { incr i; set psinit [lindex $argv $i] }
		-uart { incr i; set uart [lindex $argv $i] }
		default { puts "Unknown argument $a"; exit 1 }
	}
}

foreach f [list $cpu0 $cpu1 $bit] {
	if {![file exists $f]} {
		puts "ERROR: missing $f"
		exit 1
	}
}

puts "Connecting..."
connect
targets -set -filter {name =~ "APU*"}
rst -system
after 500
fpga -file $bit
targets -set -filter {name =~ "*Cortex-A9 MPCore #0*"}
if {[file exists $psinit]} {
	source $psinit
	ps7_init
	ps7_post_config
}

puts "Loading CPU1 $cpu1"
targets -set -filter {name =~ "*Cortex-A9 MPCore #1*"}
rst -processor
dow $cpu1

puts "Initializing AMP mailbox at 0xFFFF0000 (magic/ABI/STARTING) before releasing CPU1"
targets -set -filter {name =~ "*Cortex-A9 MPCore #0*"}
set mb_addr 0xFFFF0000
set mb_words [list 0x52504744 1 1 0]
for {set i 0} {$i < 22} {incr i} { lappend mb_words 0 }
foreach w $mb_words {
	mwr $mb_addr $w
	incr mb_addr 4
}
puts "mailbox magic=[mrd -value 0xFFFF0000] abi=[mrd -value 0xFFFF0004] state=[mrd -value 0xFFFF0008]"

targets -set -filter {name =~ "*Cortex-A9 MPCore #1*"}
con

puts "Waiting for CPU1 READY (mailbox 0xFFFF0008 worker_state)..."
set ready 0
for {set i 0} {$i < 50} {incr i} {
	after 100
	# worker_state at mailbox+8
	set st [mrd -value 0xFFFF0008]
	puts "worker_state=$st"
	if {$st eq "0x00000002" || $st eq "0x2" || $st == 2} {
		set ready 1
		break
	}
}
if {!$ready} {
	puts "ERROR: CPU1 did not reach READY. Leaving motor stopped."
	disconnect
	exit 1
}

puts "Loading CPU0 harness $cpu0"
targets -set -filter {name =~ "*Cortex-A9 MPCore #0*"}
dow $cpu0
con
puts "CPU0 running motor-safe RPGD AMP harness."
puts "Capture UART at 230400 8N1. This script does not flash QSPI."
disconnect
exit 0
