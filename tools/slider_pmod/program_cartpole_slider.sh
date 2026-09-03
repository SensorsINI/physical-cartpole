#!/bin/bash
set -e
# Same short-pole SecLoc bitstream + AMP CPU0 (show mux) + CPU1 (RPGD worker).
# CPU1 WFE-sleeps until SW0 selects RPGD.
ROOT="$(cd "$(dirname "$(readlink -f "$0")")/../.." && pwd)"
exec "${ROOT}/Firmware/Scripts/program_rpgd_amp_production.sh"
