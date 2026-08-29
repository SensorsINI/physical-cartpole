#ifndef QSPI_NVPARAMS_H
#define QSPI_NVPARAMS_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Zybo Z7-20 on-board S25FL128S (16 MiB). BOOT.BIN occupies the start of
 * flash. Hanging calibration uses two non-overlapping tail slots so a
 * hybrid-top 4 KiB fallback cannot sit inside a 64 KiB erase window.
 *
 * Record (little-endian, 16 bytes in a 256-byte page):
 *   u32 magic   'CPL1' (0x43504C31)
 *   u32 version 1
 *   float ANGLE_HANGING
 *   u32 crc32   IEEE 802.3 / zlib of the first 12 bytes
 *
 * Returns 0 on success, non-zero on skip/failure. Safe to call on Zedboard
 * or when the QSPI driver is missing: those builds stub out to failure.
 */
#define QSPI_NV_MAGIC          0x43504C31u
#define QSPI_NV_VERSION        1u
#define QSPI_NV_RECORD_BYTES   16u
#define QSPI_NV_PAGE_BYTES     256u
#define QSPI_NV_SECTOR_OFFSET  0x00FD0000u  /* last uniform 64 KiB; 64 KiB erase */
#define QSPI_NV_SUBSECTOR_OFF  0x00FFF000u  /* last 4 KiB; hybrid fallback */

int QspiNv_Init(void);
int QspiNv_LoadHanging(float *hanging_out);
int QspiNv_SaveHanging(float hanging);

#ifdef __cplusplus
}
#endif

#endif /* QSPI_NVPARAMS_H */
