"""Show AMP QSPI boot: CPU0 embeds and starts CPU1. No destination_cpu."""
from __future__ import annotations

from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
MAIN = (REPO / "Firmware" / "Src" / "CartPoleFirmware" / "main.c").read_text()
AMP_C = (REPO / "Firmware" / "Src" / "Zynq" / "amp_ipc.c").read_text()
AMP_H = (REPO / "Firmware" / "Src" / "Zynq" / "amp_ipc.h").read_text()
BUILD = (REPO / "Firmware" / "Scripts" / "build_rpgd_amp_elfs.sh").read_text()
PROD_TCL = (REPO / "Firmware" / "Scripts" / "program_rpgd_amp_production.tcl").read_text()
PROD_SH = (REPO / "Firmware" / "Scripts" / "program_rpgd_amp_production.sh").read_text()
SHOW_QSPI = (REPO / "Firmware" / "Scripts" / "program_show_qspi.sh").read_text()
BIF = (REPO / "Firmware" / "Scripts" / "cartpole_qspi.bif").read_text()
QSPI_TCL = (REPO / "Firmware" / "Scripts" / "program_qspi_boot.tcl").read_text()
QSPI_FSBL_PATCH = (
    REPO / "Firmware" / "Scripts" / "patch_qspi_fsbl_single.py"
).read_text()
SD_INSTALL = (REPO / "Firmware" / "Scripts" / "install_show_sd.sh").read_text()
CPU0_LD = (REPO / "Firmware" / "Src" / "Zynq" / "lscript_cpu0_amp.ld").read_text()
CPU1_LD = (REPO / "Firmware" / "Src" / "RPGDWorker" / "lscript_cpu1_amp.ld").read_text()
NV = (REPO / "Firmware" / "Src" / "Zynq" / "qspi_nvparams.h").read_text()
README = (REPO / "README.md").read_text()


def test_main_starts_cpu1_after_uart():
    assert "amp_ipc_load_and_start_cpu1" in MAIN
    assert MAIN.count("amp_ipc_load_and_start_cpu1();") == 2
    prod = MAIN.split("#else", 1)[1]
    assert prod.find("PC_Connection_Init") < prod.find("amp_ipc_load_and_start_cpu1")
    assert prod.find("amp_ipc_load_and_start_cpu1") < prod.find("CONTROL_Init")


def test_load_and_start_skips_only_if_worker_answers():
    assert "cpu1_worker_alive" in AMP_C
    assert "AMP_CMD_SYNC" in AMP_C.split("cpu1_worker_alive", 1)[1]
    assert "heartbeat" in AMP_C.split("cpu1_worker_alive", 1)[1]
    fn = AMP_C.split("int amp_ipc_load_and_start_cpu1(void)", 1)[1]
    assert "cpu1_blob_start" in fn
    assert "AMP_IPC_CPU1_LOAD_ADDR" in fn
    assert "Xil_ICacheInvalidateRange" in fn
    assert "amp_ipc_start_cpu1" in fn
    assert "CPU1 READY" in fn
    assert "CPU1 start FAIL" in fn
    assert "AMP_IPC_CPU1_LOAD_ADDR 0x10000000u" in AMP_H


def test_start_cpu1_flushes_vector_and_releases_reset():
    assert "0xFFFFFFE0u" in AMP_C
    regions = AMP_C.split("void amp_ipc_configure_regions(void)", 1)[1]
    regions = regions.split("void amp_ipc_dmb(void)", 1)[0]
    assert "AMP_IPC_SHARED_ADDR" in regions
    assert "NORM_WT_CACHE" in regions
    assert "Xil_SetTlbAttributes" in regions
    assert "STRONG_ORDERED" in regions
    assert "0xFFF00000" in regions
    fn = AMP_C.split("void amp_ipc_start_cpu1(uint32_t entry_addr)", 1)[1]
    fn = fn.split("#if defined(RPGD_DUAL_CORE)", 1)[0]
    assert "AMP_IPC_CPU1_VECTOR" in fn
    assert "amp_ipc_configure_regions" in fn
    assert "0xF8000244u" in fn
    assert "0x2u | 0x20u" in fn


def test_production_jtag_does_not_dow_cpu1():
    assert "dow $cpu1" not in PROD_TCL
    assert "RPGDWorker_cpu1.elf" not in PROD_TCL
    assert "dow $cpu0" in PROD_TCL
    after_dow = PROD_TCL.split("dow $cpu0", 1)[1]
    assert after_dow.find("\ncon") < after_dow.find("0xFFFF0008")
    assert "0x52504744" in after_dow
    assert "CPU0 starts CPU1" in PROD_TCL
    assert "Does not write QSPI" in PROD_TCL
    assert "same path as QSPI" in PROD_SH


def test_show_qspi_is_single_app_no_destination_cpu():
    xsct = [line for line in SHOW_QSPI.splitlines() if "program_qspi_boot.tcl" in line]
    assert xsct
    assert all("--accept-dual-qspi" not in line and "-elf1" not in line for line in xsct)
    assert "[destination_cpu" not in SHOW_QSPI
    image = BIF.split("the_ROM_image:", 1)[1]
    assert "[destination_cpu" not in image
    assert "CartPoleFirmware_rpgd_amp_cpu0.elf" in SHOW_QSPI
    assert "cartpole_short_pole_secloc.bit" in SHOW_QSPI
    assert "boot/fsbl.elf" in SHOW_QSPI
    assert "0xFD0000" in SHOW_QSPI
    assert "-boot" in SHOW_QSPI
    assert "bootgen -arch zynq" in SHOW_QSPI
    assert "SHOW_QSPI_SKIP_BUILD" in SHOW_QSPI
    assert "RPGD_AMP_PRODUCTION=1" in SHOW_QSPI


def test_qspi_programmer_keeps_dual_elf_gated():
    assert "--accept-dual-qspi" in QSPI_TCL
    assert "destination_cpu=a9-1" in QSPI_TCL
    assert "image-range erase only" in QSPI_TCL


def test_show_qspi_builds_fsbl_with_reliable_single_bit_reads():
    assert "patch_qspi_fsbl_single.py" in SHOW_QSPI
    assert 'make -C "${FSBL_DIR}"' in SHOW_QSPI
    assert "SHOW_QSPI_BUILD_ONLY" in SHOW_QSPI
    assert SHOW_QSPI.find('if [ -f "${FSBL_ZYNQ}" ]') < SHOW_QSPI.find(
        'elif [ -f "${FSBL_EXPORT}" ]'
    )
    assert "SINGLE_QSPI_CONFIG_FAST_QUAD_READ" in QSPI_FSBL_PATCH
    assert "SINGLE_QSPI_CONFIG_FAST_READ" in QSPI_FSBL_PATCH
    assert "replace(old, new, 1)" in QSPI_FSBL_PATCH


def test_sd_installer_copies_same_show_image_without_formatting():
    assert "SHOW_QSPI_BUILD_ONLY=1" in SD_INSTALL
    assert "SD_Zybo" in SD_INSTALL
    assert 'DEST="${MOUNT}/BOOT.BIN"' in SD_INSTALL
    assert "cmp -s" in SD_INSTALL
    assert "findmnt" in SD_INSTALL
    assert "vfat|fat|msdos" in SD_INSTALL
    assert "mkfs" not in SD_INSTALL
    assert "parted" not in SD_INSTALL
    assert "fdisk" not in SD_INSTALL
    assert "install_show_sd.sh" in README
    assert "JP5 = SD" in README


def test_cpu1_blob_is_linked_from_objcopy():
    assert "objcopy" in BUILD.lower() or "OBJCOPY" in BUILD
    assert "cpu1_blob.S" in BUILD
    assert ".incbin" in BUILD
    assert "cpu1_blob_start" in BUILD
    assert "Linking ${CPU1_ELF}" in BUILD
    cpu1_link = BUILD.find('echo "Linking ${CPU1_ELF}"')
    cpu0_link = BUILD.find('echo "Linking ${CPU0_ELF}"')
    assert 0 <= cpu1_link < cpu0_link
    assert "0x10000000" in BUILD
    assert "64 KiB" in BUILD or "65536" in BUILD
    assert "PT_LOAD FileSiz" in BUILD
    assert "exactly one" in BUILD


def test_amp_memory_windows():
    cpu0_mem = CPU0_LD.split("MEMORY", 1)[1].split("ENTRY", 1)[0]
    cpu1_mem = CPU1_LD.split("MEMORY", 1)[1].split("ENTRY", 1)[0]
    assert "ps7_ddr_0 : ORIGIN = 0x00100000, LENGTH = 0x0FF00000" in cpu0_mem
    assert "amp_shared : ORIGIN = 0x20000000" in cpu0_mem
    assert "ps7_ddr_0 : ORIGIN = 0x10000000, LENGTH = 0x10000000" in cpu1_mem


def test_hanging_slots_documented():
    assert "QSPI_NV_SECTOR_OFFSET  0x00FD0000u" in NV
    assert "QSPI_NV_SUBSECTOR_OFF  0x00FFF000u" in NV
    assert "0xFD0000" in BIF
    assert "0xFFF000" in BIF


def test_readme_show_qspi_procedure():
    assert "program_show_qspi.sh" in README
    assert "CartPoleFirmware_rpgd_amp_cpu0.elf" in README
    assert "16 MiB" in README
    assert "0xFD0000" in README
