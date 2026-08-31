"""QSPI hanging-record layout (must match Firmware/Src/Zynq/qspi_nvparams.c)."""
from __future__ import annotations

import struct
import zlib

# qspi_nvparams.h
QSPI_NV_MAGIC = 0x43504C31  # 'CPL1'
QSPI_NV_VERSION = 1
QSPI_NV_RECORD_BYTES = 16
QSPI_NV_PAGE_BYTES = 256
QSPI_NV_SECTOR_OFFSET = 0x00FD0000
QSPI_NV_SUBSECTOR_OFF = 0x00FFF000

ANGLE_360_DEG_IN_ADC_UNITS = 4051.4533


def crc32_ieee(data: bytes) -> int:
    """Same polynomial as qspi_nvparams.c (zlib / IEEE 802.3)."""
    return zlib.crc32(data) & 0xFFFFFFFF


def pack_record(hanging: float) -> bytes:
    header = struct.pack("<IIf", QSPI_NV_MAGIC, QSPI_NV_VERSION, hanging)
    rec = header + struct.pack("<I", crc32_ieee(header))
    assert len(rec) == QSPI_NV_RECORD_BYTES
    page = rec + b"\xff" * (QSPI_NV_PAGE_BYTES - QSPI_NV_RECORD_BYTES)
    return page


def unpack_record(page: bytes) -> float:
    if len(page) < QSPI_NV_RECORD_BYTES:
        raise ValueError("short record")
    rec = page[:QSPI_NV_RECORD_BYTES]
    magic, version, hanging, crc_got = struct.unpack("<IIfI", rec)
    crc_exp = crc32_ieee(rec[:12])
    if magic != QSPI_NV_MAGIC or version != QSPI_NV_VERSION:
        raise ValueError("bad magic/version")
    if crc_got != crc_exp:
        raise ValueError("bad crc")
    if hanging != hanging or hanging < 0.0 or hanging >= ANGLE_360_DEG_IN_ADC_UNITS:
        raise ValueError("hanging out of range")
    return hanging


def test_roundtrip_known_hanging():
    hanging = 1063.779
    page = pack_record(hanging)
    assert len(page) == QSPI_NV_PAGE_BYTES
    assert page[QSPI_NV_RECORD_BYTES:] == b"\xff" * (QSPI_NV_PAGE_BYTES - QSPI_NV_RECORD_BYTES)
    got = unpack_record(page)
    assert abs(got - hanging) < 1e-3


def test_crc_matches_bit_algorithm():
    header = struct.pack("<IIf", QSPI_NV_MAGIC, QSPI_NV_VERSION, 1063.779)
    crc_zlib = crc32_ieee(header)
    crc = 0xFFFFFFFF
    for byte in header:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xEDB88320
            else:
                crc >>= 1
    crc ^= 0xFFFFFFFF
    assert crc == crc_zlib


def test_blank_flash_rejected():
    blank = b"\xff" * QSPI_NV_PAGE_BYTES
    try:
        unpack_record(blank)
    except ValueError:
        return
    raise AssertionError("blank 0xFF must not unpack")


def test_bad_crc_rejected():
    page = bytearray(pack_record(1063.779))
    page[12] ^= 0xFF
    try:
        unpack_record(bytes(page))
    except ValueError as exc:
        assert "crc" in str(exc)
        return
    raise AssertionError("tampered crc must not unpack")


def test_out_of_range_rejected():
    page = pack_record(-1.0)
    try:
        unpack_record(page)
    except ValueError:
        return
    raise AssertionError("negative hanging must not unpack")


def test_firmware_circle_bound():
    hanging = ANGLE_360_DEG_IN_ADC_UNITS - 0.01
    assert abs(unpack_record(pack_record(hanging)) - hanging) < 1e-3
    try:
        unpack_record(pack_record(ANGLE_360_DEG_IN_ADC_UNITS + 1.0))
    except ValueError:
        return
    raise AssertionError("hanging at or above ANGLE_360 must not unpack")


def test_offsets_non_overlapping_and_leave_room_for_bootbin():
    assert QSPI_NV_SECTOR_OFFSET == 16 * 1024 * 1024 - 192 * 1024
    assert QSPI_NV_SUBSECTOR_OFF == 16 * 1024 * 1024 - 4 * 1024
    assert QSPI_NV_SECTOR_OFFSET + 64 * 1024 <= QSPI_NV_SUBSECTOR_OFF
