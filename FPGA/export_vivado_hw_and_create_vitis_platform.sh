#!/usr/bin/env bash
set -euo pipefail

# Export the Zedboard Vivado hardware (XSA with bitstream), create a Vitis
# platform, create the Vitis applications, populate their sources with relative
# symlinks into Firmware/Src, and build the ELFs.

VIVADO_PROJECT_NAME="${VIVADO_PROJECT_NAME:-CartpoleDriverZynq_AXIS_Zedboard}"
PLATFORM_NAME="${PLATFORM_NAME:-cartpole_zedboard}"
PROC="${PROC:-ps7_cortexa9_0}"
DOMAIN_NAME="${DOMAIN_NAME:-standalone_domain}"
: "${APPS:=CartPoleFirmware Embedded_Controller}"
read -r -a APPS <<< "${APPS}"

XILINX_VERSION="${XILINX_VERSION:-2020.1}"
XILINX_ROOT="${XILINX_ROOT:-/media/santiago/Data/tools/Xilinx}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
VIVADO_PROJECTS_ROOT="${SCRIPT_DIR}/VivadoProjects"
FIRMWARE_DIR="${REPO_ROOT}/Firmware"
VITIS_WS="${VITIS_WS:-${FIRMWARE_DIR}/VitisProjects}"

PROJECT_DIR="${VIVADO_PROJECTS_ROOT}/${VIVADO_PROJECT_NAME}"
XPR_PATH="${PROJECT_DIR}/${VIVADO_PROJECT_NAME}.xpr"
XSA_PATH="${VITIS_WS}/${PLATFORM_NAME}.xsa"
PLATFORM_XPFM="${VITIS_WS}/${PLATFORM_NAME}/export/${PLATFORM_NAME}/${PLATFORM_NAME}.xpfm"

export PATH="${XILINX_ROOT}/Vivado/${XILINX_VERSION}/bin:${XILINX_ROOT}/Vitis/${XILINX_VERSION}/bin:${PATH}"

# Vitis 2020.1 xsct needs an X display. Use the live desktop display when the
# shell has none and Xvfb is not installed.
if [ -z "${DISPLAY:-}" ] && ! command -v Xvfb >/dev/null 2>&1 && [ -S /tmp/.X11-unix/X0 ]; then
  export DISPLAY=":0"
  [ -z "${XAUTHORITY:-}" ] && [ -f "${HOME}/.Xauthority" ] && export XAUTHORITY="${HOME}/.Xauthority"
fi

if ! command -v vivado >/dev/null 2>&1; then
  echo "ERROR: vivado not found in PATH" >&2
  exit 1
fi
if ! command -v xsct >/dev/null 2>&1; then
  echo "ERROR: xsct not found in PATH" >&2
  exit 1
fi
if [ ! -f "${XPR_PATH}" ]; then
  echo "ERROR: Vivado project not found: ${XPR_PATH}" >&2
  exit 1
fi

mkdir -p "${VITIS_WS}"

echo "DEBUG PATHS:"
echo "  REPO_ROOT           = ${REPO_ROOT}"
echo "  VIVADO_PROJECT_NAME = ${VIVADO_PROJECT_NAME}"
echo "  XPR_PATH            = ${XPR_PATH}"
echo "  PLATFORM_NAME       = ${PLATFORM_NAME}"
echo "  VITIS_WS            = ${VITIS_WS}"
echo "  XSA_PATH            = ${XSA_PATH}"
echo "  PLATFORM_XPFM       = ${PLATFORM_XPFM}"
echo "  APPS                = ${APPS[*]}"
echo

# -----------------------------------------------------------------------------
# Export hardware with bitstream.
# -----------------------------------------------------------------------------
vivado_tcl="$(mktemp -t export_hw_XXXXXXXX.tcl)"
cat > "${vivado_tcl}" <<'TCL'
open_project $::env(XPR)
if {[llength [get_runs impl_1]] == 0} {
  puts "ERROR: impl_1 run not found in project."
  exit 1
}
catch {open_run impl_1}
set run_dir [get_property DIRECTORY [get_runs impl_1]]
set impl_dir $run_dir
if {[file tail $run_dir] ne "impl_1"} { set impl_dir [file join $run_dir "impl_1"] }
set bitfiles [glob -nocomplain -directory $impl_dir "*.bit"]
if {[llength $bitfiles] == 0} {
  puts "INFO: Bitstream missing; launching impl_1 to write_bitstream."
  launch_runs impl_1 -to_step write_bitstream -jobs 8 -force
  wait_on_run impl_1
  set run_dir [get_property DIRECTORY [get_runs impl_1]]
  set impl_dir $run_dir
  if {[file tail $run_dir] ne "impl_1"} { set impl_dir [file join $run_dir "impl_1"] }
  set bitfiles [glob -nocomplain -directory $impl_dir "*.bit"]
}
if {[llength $bitfiles] == 0} {
  puts "ERROR: Bitstream not found."
  exit 1
}
puts "Found bitstream: [lindex $bitfiles 0]"
write_hw_platform -fixed -include_bit -force -file $::env(XSA_PATH)
puts "Hardware export complete: $::env(XSA_PATH)"
exit 0
TCL

if [ -f "${XSA_PATH}" ] && [ "${SKIP_EXPORT:-0}" = "1" ]; then
  echo "==> SKIP_EXPORT=1 and XSA present, reusing: ${XSA_PATH}"
else
  echo "==> Exporting hardware from: ${XPR_PATH}"
  pushd "${PROJECT_DIR}" >/dev/null
  XPR="${XPR_PATH}" XSA_PATH="${XSA_PATH}" vivado -mode batch -source "${vivado_tcl}" -log vivado.log -journal vivado.jou | cat
  popd >/dev/null
fi
rm -f "${vivado_tcl}"

if [ ! -f "${XSA_PATH}" ]; then
  echo "ERROR: XSA not produced: ${XSA_PATH}" >&2
  exit 1
fi

# -----------------------------------------------------------------------------
# Create platform/apps, symlink sources, build. Keep this in one xsct session:
# Vitis 2020.1 on this machine does not reliably rediscover apps/platforms across
# fresh xsct sessions. The first app creates the platform from XSA; later apps
# reuse the same platform/domain.
# -----------------------------------------------------------------------------
SELECTED_APPS=()
for app in "${APPS[@]}"; do
  if [ -d "${REPO_ROOT}/Firmware/Src/${app}" ]; then
    SELECTED_APPS+=("${app}")
  else
    echo "WARN: skipping ${app}; source directory missing"
  fi
done
if [ "${#SELECTED_APPS[@]}" -eq 0 ]; then
  echo "ERROR: no apps selected" >&2
  exit 1
fi

xsct_tcl="$(mktemp -t vitis_all_XXXXXXXX.tcl)"
cat > "${xsct_tcl}" <<'TCL'
set ws    $::env(VITIS_WS)
set xsa   $::env(XSA_PATH)
set proc  $::env(PROC)
set pname $::env(PLATFORM_NAME)
set dom   $::env(DOMAIN_NAME)
set fw    $::env(FIRMWARE_DIR)
set xpfm  $::env(PLATFORM_XPFM)
set apps  [split [string trim $::env(APPS_LIST)]]

setws $ws
set existing_apps {}
catch {set existing_apps [app list]}
proc has_item {name lst} { return [expr {[lsearch -exact $lst $name] >= 0}] }
set platform_ready [file exists $xpfm]

# If the platform already exists, refresh it from the (possibly re-exported)
# XSA. Without this, a rebuilt bitstream with changed IP registers leaves the
# BSP stale: headers/libxil.a keep the old driver and app linking fails.
# 'bsp regenerate' is required; 'platform generate' alone does not re-extract
# driver sources from the updated hardware.
if {$platform_ready} {
  puts "Updating existing platform '$pname' from XSA"
  platform read "$ws/$pname/platform.spr"
  platform config -updatehw $xsa
  domain active $dom
  bsp regenerate
  platform generate
}

foreach app $apps {
  if {[has_item $app $existing_apps]} {
    puts "Application '$app' already exists."
  } elseif {!$platform_ready} {
    puts "Creating '$app' and platform '$pname' from XSA"
    app create -name $app -hw $xsa -proc $proc -os standalone -template "Empty Application"
    set platform_ready 1
  } else {
    puts "Creating '$app' on existing platform '$pname'"
    app create -name $app -platform $pname -domain $dom -template "Empty Application"
  }
}

foreach app $apps {
  set asrc "$ws/$app/src"
  exec bash -c "mkdir -p \"$asrc\"; find \"$asrc\" -mindepth 1 ! -name lscript.ld -exec rm -rf {} + 2>/dev/null; true"
  set rc [catch {exec bash -c "cd \"$fw\" && ./create_symlinks.sh \"./Src/$app ./VitisProjects/$app/src\" \"./Src/General ./VitisProjects/$app/src\" \"./Src/Zynq ./VitisProjects/$app/src/Zynq\""} out]
  puts $out
  if {$rc} { puts "WARN: symlink step for '$app' returned non-zero" }
}

set failed {}
foreach app $apps {
  catch {app config -name $app -add include-path {../src}}
  catch {app config -name $app -add include-path {../src/Zynq}}
  catch {app config -name $app -add libraries {m}}
  puts "Building '$app'..."
  if {[catch {app build -name $app} err]} {
    puts "BUILD_FAIL $app: $err"
    lappend failed $app
  } else {
    puts "BUILD_OK $app"
  }
}
if {[llength $failed] > 0} { puts "APPS_WITH_BUILD_ERRORS: $failed" }
exit 0
TCL

VITIS_WS="${VITIS_WS}" XSA_PATH="${XSA_PATH}" PROC="${PROC}" PLATFORM_NAME="${PLATFORM_NAME}" \
  DOMAIN_NAME="${DOMAIN_NAME}" FIRMWARE_DIR="${FIRMWARE_DIR}" PLATFORM_XPFM="${PLATFORM_XPFM}" \
  APPS_LIST="${SELECTED_APPS[*]}" xsct "${xsct_tcl}" | cat
rm -f "${xsct_tcl}"

echo
if [ -f "${PLATFORM_XPFM}" ]; then
  echo "Platform: ${PLATFORM_XPFM}"
else
  echo "WARN: platform xpfm not found: ${PLATFORM_XPFM}"
fi
for app in "${SELECTED_APPS[@]}"; do
  elf="${VITIS_WS}/${app}/Debug/${app}.elf"
  if [ -f "${elf}" ]; then
    echo "  OK  ${app}: ${elf}"
  else
    echo "  ERR ${app}: ELF not found (${elf})"
  fi
done

echo "Done. Platform '${PLATFORM_NAME}' and apps [${SELECTED_APPS[*]}] under: ${VITIS_WS}"
