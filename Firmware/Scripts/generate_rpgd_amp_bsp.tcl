set script_dir [file dirname [file normalize [info script]]]
set repo_root [file normalize [file join $script_dir ../..]]
set ws [file normalize [file join $repo_root Firmware AmpWorkspace]]
setws $ws
platform read [file join $ws cartpole_rpgd_amp platform.spr]
platform active cartpole_rpgd_amp
platform generate
puts "OK: regenerated AMP BSPs"
exit 0
