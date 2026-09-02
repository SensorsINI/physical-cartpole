#!/bin/bash
set -e
# Same bitstream + AMP CPU0/CPU1 as production. There is no single-core
# Debug ELF flash path; CPU1 WFE-sleeps until RPGD needs it.
ROOT="$(cd "$(dirname "$(readlink -f "$0")")/../.." && pwd)"
exec "${ROOT}/Firmware/Scripts/program_rpgd_amp_production.sh"
