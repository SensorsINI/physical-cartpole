# Add a CPU1 AMP domain to the live cartpole_zybo_secloc2026 platform
# (PL buttons / slider XSA). Does not create a second hardware platform.
# CPU0 standalone_domain is left as-is (no USE_AMP) so Debug LSTM stays valid.
#
# Usage:
#   xsct Firmware/Scripts/setup_rpgd_amp_platform.tcl

set script_dir [file dirname [file normalize [info script]]]
set repo_root [file normalize [file join $script_dir ../..]]
set ws [file join $repo_root Firmware VitisProjects]
set plat_name cartpole_zybo_secloc2026
set spr [file join $ws $plat_name platform.spr]

if {![file exists $spr]} {
	puts "ERROR: $spr not found"
	exit 1
}

puts "workspace=$ws"
puts "platform=$plat_name"
setws $ws
platform read $spr
platform active $plat_name

set domains [domain list]
puts "domains:\n$domains"

set have_cpu1 0
if {[string match "*standalone_domain_cpu1*" $domains]} {
	set have_cpu1 1
}

if {!$have_cpu1} {
	if {[catch {domain create -name standalone_domain_cpu1 -os standalone -proc ps7_cortexa9_1} err]} {
		puts "ERROR: domain create cpu1: $err"
		exit 1
	}
	puts "created standalone_domain_cpu1"
} else {
	puts "standalone_domain_cpu1 already present"
}

domain active standalone_domain_cpu1
catch {bsp config USE_AMP "1"}
catch {bsp config extra_compiler_flags "-g -ffunction-sections -fdata-sections -Wall -Wextra -mcpu=cortex-a9 -mfpu=vfpv3 -mfloat-abi=hard -DUSE_AMP=1"}

domain active standalone_domain
puts "CPU0 domain left without USE_AMP"

platform write
if {[catch {platform generate} gerr]} {
	puts "ERROR: platform generate: $gerr"
	exit 1
}
puts "OK: $plat_name has CPU0 standalone + CPU1 AMP domains"
exit 0
