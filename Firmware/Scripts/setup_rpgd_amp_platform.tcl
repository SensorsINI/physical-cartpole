# Create / regenerate a dual-core AMP platform without mutating the
# production cartpole_zybo_secloc2026 platform.
#
# Usage:
#   xsct Firmware/Scripts/setup_rpgd_amp_platform.tcl
# Optional env:
#   RPGD_XSA, RPGD_VITIS_WS, RPGD_AMP_PLATFORM_NAME

set script_dir [file dirname [file normalize [info script]]]
set repo_root [file normalize [file join $script_dir ../..]]
set ws [file join $repo_root Firmware AmpWorkspace]
if {[info exists ::env(RPGD_VITIS_WS)]} { set ws $::env(RPGD_VITIS_WS) }
set ws [file normalize $ws]
set xsa [file join $repo_root Firmware VitisProjects cartpole_zybo_secloc2026.xsa]
if {[info exists ::env(RPGD_XSA)]} { set xsa $::env(RPGD_XSA) }
set xsa [file normalize $xsa]
set plat_name cartpole_rpgd_amp
if {[info exists ::env(RPGD_AMP_PLATFORM_NAME)]} { set plat_name $::env(RPGD_AMP_PLATFORM_NAME) }

if {![file exists $xsa]} {
	puts "ERROR: XSA not found: $xsa"
	exit 1
}

puts "workspace=$ws"
puts "xsa=$xsa"
puts "platform=$plat_name"
file mkdir $ws
setws $ws

if {[catch {platform remove $plat_name} err]} {
	puts "platform remove skipped: $err"
}

platform create -name $plat_name -hw $xsa -os standalone -proc ps7_cortexa9_0 -out $ws
platform write
platform active $plat_name

domain active standalone_domain
catch {bsp config USE_AMP "1"}
catch {bsp config extra_compiler_flags "-g -ffunction-sections -fdata-sections -Wall -Wextra -mcpu=cortex-a9 -mfpu=vfpv3 -mfloat-abi=hard -DUSE_AMP=1"}

if {[catch {domain create -name standalone_domain_cpu1 -os standalone -proc ps7_cortexa9_1} err]} {
	puts "domain create cpu1: $err"
}
domain active standalone_domain_cpu1
catch {bsp config USE_AMP "1"}
catch {bsp config extra_compiler_flags "-g -ffunction-sections -fdata-sections -Wall -Wextra -mcpu=cortex-a9 -mfpu=vfpv3 -mfloat-abi=hard -DUSE_AMP=1"}

platform generate
puts "OK: generated $plat_name with CPU0 + CPU1 AMP domains"
exit 0
