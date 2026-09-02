# Regenerate BSPs on the live secloc / PL-buttons platform.
set script_dir [file dirname [file normalize [info script]]]
set repo_root [file normalize [file join $script_dir ../..]]
set ws [file normalize [file join $repo_root Firmware VitisProjects]]
set spr [file join $ws cartpole_zybo_secloc2026 platform.spr]
setws $ws
platform read $spr
platform active cartpole_zybo_secloc2026
platform generate
puts "OK: regenerated cartpole_zybo_secloc2026 BSPs"
exit 0
