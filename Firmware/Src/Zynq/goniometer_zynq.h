#ifndef __GONIOMETER_ZYNQ_H_
#define __GONIOMETER_ZYNQ_H_

#include "xsysmon.h"
#include "xadcps.h"
#include <stdio.h>
#include "param_pkg.h"

#define XADC_DEVICE_ID XPAR_PS7_XADC_0_DEVICE_ID

#include "xmedian_filter.h"
#define MEDIAN_FILTER_DEVICE_ID XPAR_CARTPOLEINTERFACE_MEDIAN_FILTER_0_DEVICE_ID

// Hardware XADC filter modes. These must match FPGA/CustomIPs/median_filter_hls/median_functions.h.
#define HARDWARE_FILTER_MODE_RAW 0
#define HARDWARE_FILTER_MODE_MEDIAN 1
#define HARDWARE_FILTER_MODE_TRIMMED_MEAN 2

// Boot default chosen from the 2026-07-08 hardware campaign (see
// Boot default (chosen from hardware campaign): trimmed mean 63/7 gives ~0.16 LSB
// static noise (vs 0.18 median) and ~5x smoother derivatives during motion,
// while trimming 7 samples per side keeps median-grade glitch rejection.
// Dead-zone transitions are covered by the dz_* tracking registers.
#define HARDWARE_FILTER_WINDOW_SIZE 63
#define HARDWARE_FILTER_TRIM_COUNT 7
#define HARDWARE_FILTER_MODE_DEFAULT HARDWARE_FILTER_MODE_TRIMMED_MEAN

// Dead-zone (rail) detection thresholds, 16-bit filter domain (12-bit code x16).
// A raw sample <= LOW or >= HIGH means the wiper is at/near a track end, i.e.
// inside or entering the potentiometer dead zone. 12-bit codes 20 and 4090
// were chosen from swing recordings: in-gap readings settle at ~8 (low side)
// and rail at 4095 (high side), while valid on-track readings stay clear of both.
#define HARDWARE_FILTER_RAIL_LOW  (20 * 16)
#define HARDWARE_FILTER_RAIL_HIGH (4090 * 16)

// dz_status bits (must match FPGA/CustomIPs/median_filter_hls/median_filter.h)
#define HARDWARE_DZ_STATUS_LOW_RAIL  0x1
#define HARDWARE_DZ_STATUS_HIGH_RAIL 0x2

typedef struct {
	unsigned short status;      // rail contact of the latest raw sample (bits above)
	unsigned short window;      // near-rail samples currently in the filter window (0..window_size)
	unsigned short age;         // samples (~2.2 us each) since last rail contact; 0xFFFF = none recently
	unsigned int   low_count;   // cumulative low-rail sample count (wraps at 2^32)
	unsigned int   high_count;  // cumulative high-rail sample count
} GoniometerDeadZoneInfo;


void             Goniometer_Init(void);
unsigned short 	Goniometer_Read(void);
unsigned short 	Goniometer_ReadRaw(void);

// Runtime reconfiguration of the FPGA filter block (window size, trim count,
// mode as defined above). Used by CMD_SET_ANGLE_FILTER when the driver
// overrides the boot default at startup.
void             Goniometer_SetFilter(unsigned short window_size, unsigned short trim_count, unsigned short filter_mode);

// Read the hardware dead-zone tracking registers (control-loop latch path).
void             Goniometer_ReadDeadZone(GoniometerDeadZoneInfo * info);

#endif /*__GONIOMETER_ZYNQ_H_*/
