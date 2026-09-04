"""QSPI angle-calibration record (must match qspi_nvparams.c)."""
from __future__ import annotations

import struct
import zlib
from pathlib import Path

# qspi_nvparams.h
QSPI_NV_MAGIC = 0x43504C31  # 'CPL1'
QSPI_NV_VERSION = 2
QSPI_NV_RECORD_BYTES = 20
QSPI_NV_PAGE_BYTES = 256
QSPI_NV_SECTOR_OFFSET = 0x00FD0000
QSPI_NV_SUBSECTOR_OFF = 0x00FFF000

REPO = Path(__file__).resolve().parents[1]
NV_HEADER = (REPO / "Firmware/Src/Zynq/qspi_nvparams.h").read_text()
NV_IMPL = (REPO / "Firmware/Src/Zynq/qspi_nvparams.c").read_text()


def crc32_ieee(data: bytes) -> int:
    """Same polynomial as qspi_nvparams.c (zlib / IEEE 802.3)."""
    return zlib.crc32(data) & 0xFFFFFFFF


def pack_record(hanging: float, angle_360: float) -> bytes:
    header = struct.pack(
        "<IIff", QSPI_NV_MAGIC, QSPI_NV_VERSION, hanging, angle_360
    )
    rec = header + struct.pack("<I", crc32_ieee(header))
    assert len(rec) == QSPI_NV_RECORD_BYTES
    page = rec + b"\xff" * (QSPI_NV_PAGE_BYTES - QSPI_NV_RECORD_BYTES)
    return page


def unpack_record(page: bytes) -> tuple[float, float]:
    if len(page) < QSPI_NV_RECORD_BYTES:
        raise ValueError("short record")
    rec = page[:QSPI_NV_RECORD_BYTES]
    magic, version, hanging, angle_360, crc_got = struct.unpack("<IIffI", rec)
    crc_exp = crc32_ieee(rec[:16])
    if magic != QSPI_NV_MAGIC or version != QSPI_NV_VERSION:
        raise ValueError("bad magic/version")
    if crc_got != crc_exp:
        raise ValueError("bad crc")
    if angle_360 != angle_360 or not 2048.0 <= angle_360 <= 8192.0:
        raise ValueError("circle out of range")
    if hanging != hanging or hanging < 0.0 or hanging >= angle_360:
        raise ValueError("hanging out of range")
    return hanging, angle_360


def test_roundtrip_calibration_pair():
    hanging, circle = 1063.779, 4068.73
    page = pack_record(hanging, circle)
    assert len(page) == QSPI_NV_PAGE_BYTES
    assert page[QSPI_NV_RECORD_BYTES:] == b"\xff" * (QSPI_NV_PAGE_BYTES - QSPI_NV_RECORD_BYTES)
    got_hanging, got_circle = unpack_record(page)
    assert abs(got_hanging - hanging) < 1e-3
    assert abs(got_circle - circle) < 1e-3


def test_firmware_schema_and_api_store_both_values():
    assert "#define QSPI_NV_VERSION        2u" in NV_HEADER
    assert "#define QSPI_NV_RECORD_BYTES   20u" in NV_HEADER
    assert "QspiNv_LoadCalibration(float *hanging_out, float *angle_360_out)" in NV_HEADER
    assert "QspiNv_SaveCalibration(float hanging, float angle_360)" in NV_HEADER
    assert "memcpy(&page[8], &hanging, 4);" in NV_IMPL
    assert "memcpy(&page[12], &angle_360, 4);" in NV_IMPL
    assert "put_u32_le(&page[16], crc32_ieee(page, 16));" in NV_IMPL


def test_crc_matches_bit_algorithm():
    header = struct.pack(
        "<IIff", QSPI_NV_MAGIC, QSPI_NV_VERSION, 1063.779, 4068.73
    )
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
    page = bytearray(pack_record(1063.779, 4068.73))
    page[16] ^= 0xFF
    try:
        unpack_record(bytes(page))
    except ValueError as exc:
        assert "crc" in str(exc)
        return
    raise AssertionError("tampered crc must not unpack")


def test_out_of_range_rejected():
    page = pack_record(-1.0, 4068.73)
    try:
        unpack_record(page)
    except ValueError:
        return
    raise AssertionError("negative hanging must not unpack")


def test_hanging_must_fit_stored_circle():
    circle = 4068.73
    hanging = circle - 0.01
    got_hanging, got_circle = unpack_record(pack_record(hanging, circle))
    assert abs(got_hanging - hanging) < 1e-3
    assert abs(got_circle - circle) < 1e-3
    try:
        unpack_record(pack_record(circle + 1.0, circle))
    except ValueError:
        return
    raise AssertionError("hanging at or above ANGLE_360 must not unpack")


def test_implausible_circle_rejected():
    for circle in (0.0, 2047.0, 8193.0, float("nan")):
        try:
            unpack_record(pack_record(1000.0, circle))
        except ValueError:
            continue
        raise AssertionError(f"implausible circle {circle!r} must not unpack")


def test_version_1_hanging_only_record_rejected():
    old_header = struct.pack("<IIf", QSPI_NV_MAGIC, 1, 1063.779)
    old_record = old_header + struct.pack("<I", crc32_ieee(old_header))
    page = old_record + b"\xff" * (QSPI_NV_PAGE_BYTES - len(old_record))
    try:
        unpack_record(page)
    except ValueError as exc:
        assert "magic/version" in str(exc)
        return
    raise AssertionError("v1 record has no angle circle and must be rejected")


def test_offsets_non_overlapping_and_leave_room_for_bootbin():
    assert QSPI_NV_SECTOR_OFFSET == 16 * 1024 * 1024 - 192 * 1024
    assert QSPI_NV_SUBSECTOR_OFF == 16 * 1024 * 1024 - 4 * 1024
    assert QSPI_NV_SECTOR_OFFSET + 64 * 1024 <= QSPI_NV_SUBSECTOR_OFF
