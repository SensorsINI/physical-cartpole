#include "goniometer_zynq.h"
#include "math.h"
#include <stdlib.h>

XAdcPs XADC_Driver_Instance ;
XAdcPs *XADC_Driver_Ptr = &XADC_Driver_Instance;

XMedian_filter XMedian_filter_Instance;
XMedian_filter * XMedian_filter_Ptr = &XMedian_filter_Instance;

void Goniometer_Init(void)
{

	XMedian_filter_Initialize(XMedian_filter_Ptr, MEDIAN_FILTER_DEVICE_ID);
	XMedian_filter_Set_window_size(XMedian_filter_Ptr, HARDWARE_FILTER_WINDOW_SIZE);
	XMedian_filter_Set_trim_count(XMedian_filter_Ptr, HARDWARE_FILTER_TRIM_COUNT);
	XMedian_filter_Set_filter_mode(XMedian_filter_Ptr, HARDWARE_FILTER_MODE_DEFAULT);
	XMedian_filter_Set_rail_low(XMedian_filter_Ptr, HARDWARE_FILTER_RAIL_LOW);
	XMedian_filter_Set_rail_high(XMedian_filter_Ptr, HARDWARE_FILTER_RAIL_HIGH);

    XAdcPs_Config* cfg = XAdcPs_LookupConfig ( XADC_DEVICE_ID ) ;

    XAdcPs_CfgInitialize(& XADC_Driver_Instance, cfg , cfg -> BaseAddress );
    XAdcPs_SetSequencerMode(& XADC_Driver_Instance , XADCPS_SEQ_MODE_SAFE);
    // The goniometer is wired to VAUX0 (see cartpole_pinout_zed.xdc and the
    // xadc_wiz single-channel selection VAUXP0_VAUXN0 in the block design).
    XAdcPs_SetSeqChEnables(& XADC_Driver_Instance , XADCPS_SEQ_CH_VPVN | XADCPS_SEQ_CH_AUX00);
    XAdcPs_SetSeqInputMode(& XADC_Driver_Instance , XADCPS_SEQ_CH_AUX00);
    XAdcPs_SetAvg(& XADC_Driver_Instance , XADCPS_AVG_0_SAMPLES ) ;
    //Single Channel
    XAdcPs_SetSequencerMode (& XADC_Driver_Instance , XADCPS_SEQ_MODE_SINGCHAN ) ;
}

unsigned short Goniometer_Read(void)
{
	// Set conversion sequence		 
//	unsigned short volt_raw = XAdcPs_GetAdcData(XADC_Driver_Ptr, XADCPS_CH_AUX_MAX);
	unsigned short volt_raw = XMedian_filter_Get_filtered_o(XMedian_filter_Ptr);
//	float volt_f = XSysMon_RawToExtVoltage(volt_raw);
	return volt_raw/16;
}

unsigned short Goniometer_ReadRaw(void)
{
	unsigned short volt_raw = XMedian_filter_Get_raw_o(XMedian_filter_Ptr);
	return volt_raw/16;
}

void Goniometer_SetFilter(unsigned short window_size, unsigned short trim_count, unsigned short filter_mode)
{
	XMedian_filter_Set_window_size(XMedian_filter_Ptr, window_size);
	XMedian_filter_Set_trim_count(XMedian_filter_Ptr, trim_count);
	XMedian_filter_Set_filter_mode(XMedian_filter_Ptr, filter_mode);
}

void Goniometer_ReadDeadZone(GoniometerDeadZoneInfo * info)
{
	info->status     = (unsigned short)XMedian_filter_Get_dz_status_o(XMedian_filter_Ptr);
	info->window     = (unsigned short)XMedian_filter_Get_dz_window_o(XMedian_filter_Ptr);
	info->age        = (unsigned short)XMedian_filter_Get_dz_age_o(XMedian_filter_Ptr);
	info->low_count  = XMedian_filter_Get_dz_low_count(XMedian_filter_Ptr);
	info->high_count = XMedian_filter_Get_dz_high_count(XMedian_filter_Ptr);
}
