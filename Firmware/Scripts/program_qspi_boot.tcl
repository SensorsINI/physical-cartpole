# Program Zybo Z7-20 QSPI with BOOT.BIN at offset 0.
#
# Erases only the image range (not the whole 16 MiB). Hanging calibration
# at 0xFD0000 (64 KiB) / 0xFFF000 (4 KiB fallback) is preserved as long as
# BOOT.BIN stays well under ~15.8 MiB.
#
# Usage (xsct):
#   xsct program_qspi_boot.tcl -boot BOOT.BIN -fsbl fsbl.elf
#   xsct program_qspi_boot.tcl -fsbl fsbl.elf -bit system.bit -elf CartPoleFirmware.elf
#
# Usage (program_flash directly):
#   program_flash -f BOOT.BIN -offset 0 -flash_type qspi-x4-single -fsbl fsbl.elf -verify
#
# Then power off, set JP5 to the two center pins labeled QSPI, power on.

set script_dir [file dirname [file normalize [info script]]]

proc usage {} {
	puts "Usage:"
	puts "  xsct program_qspi_boot.tcl -boot BOOT.BIN -fsbl fsbl.elf"
	puts "  xsct program_qspi_boot.tcl -fsbl fsbl.elf -bit system.bit -elf app.elf \[-o BOOT.BIN\]"
	puts "  xsct program_qspi_boot.tcl -fsbl fsbl.elf -bit system.bit -elf cpu0.elf -elf1 cpu1.elf --accept-dual-qspi"
	exit 1
}

set boot ""
set fsbl ""
set bit ""
set elf ""
set elf1 ""
set accept_dual 0
set out [file join $script_dir BOOT.BIN]
set flash_type "qspi-x4-single"

for {set i 0} {$i < $argc} {incr i} {
	set a [lindex $argv $i]
	switch -- $a {
		-boot - --boot {
			incr i
			set boot [lindex $argv $i]
		}
		-fsbl - --fsbl {
			incr i
			set fsbl [lindex $argv $i]
		}
		-bit - --bit {
			incr i
			set bit [lindex $argv $i]
		}
		-elf - --elf {
			incr i
			set elf [lindex $argv $i]
		}
		-elf1 - --elf1 {
			incr i
			set elf1 [lindex $argv $i]
		}
		--accept-dual-qspi {
			set accept_dual 1
		}
		-o - --out {
			incr i
			set out [lindex $argv $i]
		}
		-h - --help {
			usage
		}
		default {
			puts "Unknown argument: $a"
			usage
		}
	}
}

if {$fsbl eq ""} {
	puts "ERROR: -fsbl is required (Zynq program_flash uses FSBL to init PS QSPI)."
	usage
}
if {![file exists $fsbl]} {
	puts "ERROR: FSBL not found: $fsbl"
	exit 1
}

if {$boot eq ""} {
	if {$bit eq "" || $elf eq ""} {
		puts "ERROR: give -boot BOOT.BIN, or -bit and -elf to run bootgen."
		usage
	}
	if {![file exists $bit]} {
		puts "ERROR: bitstream not found: $bit"
		exit 1
	}
	if {![file exists $elf]} {
		puts "ERROR: ELF not found: $elf"
		exit 1
	}
	if {$elf1 ne ""} {
		if {!$accept_dual} {
			puts "ERROR: dual-ELF QSPI requires --accept-dual-qspi after Stage A-D acceptance."
			puts "NV-parameter sectors 0x00FD0000 and 0x00FFF000 are preserved only if BOOT.BIN stays below them."
			exit 1
		}
		if {![file exists $elf1]} {
			puts "ERROR: CPU1 ELF not found: $elf1"
			exit 1
		}
	}
	set bif [file join $script_dir cartpole_qspi.generated.bif]
	set fh [open $bif w]
	puts $fh "the_ROM_image:"
	puts $fh "\{"
	puts $fh "\t\[bootloader\] [file normalize $fsbl]"
	puts $fh "\t[file normalize $bit]"
	if {$elf1 eq ""} {
		puts $fh "\t[file normalize $elf]"
	} else {
		puts $fh "\t\[destination_cpu=a9-0\] [file normalize $elf]"
		puts $fh "\t\[destination_cpu=a9-1\] [file normalize $elf1]"
	}
	puts $fh "\}"
	close $fh
	puts "bootgen -> $out"
	set rc [catch {exec bootgen -arch zynq -image $bif -w on -o i $out} result]
	puts $result
	if {$rc != 0} {
		puts "ERROR: bootgen failed"
		exit 1
	}
	set boot $out
}

if {![file exists $boot]} {
	puts "ERROR: BOOT.BIN not found: $boot"
	exit 1
}

puts "program_flash $boot offset 0 type $flash_type (image-range erase only, not full chip)"
# Do not pass a full-chip erase flag. 2020.1 program_flash erases the image size.
set rc [catch {
	exec program_flash -f $boot -offset 0 -flash_type $flash_type -fsbl $fsbl -verify
} result]
puts $result
if {$rc != 0} {
	puts "ERROR: program_flash failed"
	exit 1
}
puts "OK. Power off, JP5 = QSPI (two center pins), power on."
