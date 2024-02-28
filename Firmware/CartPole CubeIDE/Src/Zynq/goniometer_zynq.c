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
	XMedian_filter_Set_window_size(XMedian_filter_Ptr, HARDWARE_MEDIAN);

    XAdcPs_Config* cfg = XAdcPs_LookupConfig ( XADC_DEVICE_ID ) ;

    XAdcPs_CfgInitialize(& XADC_Driver_Instance, cfg , cfg -> BaseAddress );
    XAdcPs_SetSequencerMode(& XADC_Driver_Instance , XADCPS_SEQ_MODE_SINGCHAN);
    XAdcPs_SetSequencerMode(& XADC_Driver_Instance , XADCPS_SEQ_MODE_SAFE);
    XAdcPs_SetSeqChEnables(& XADC_Driver_Instance , XADCPS_SEQ_CH_VPVN | XADCPS_SEQ_CH_AUX15) ;
    XAdcPs_SetSeqInputMode(& XADC_Driver_Instance , XADCPS_SEQ_CH_AUX15) ;
    XAdcPs_SetAvg(& XADC_Driver_Instance , XADCPS_AVG_0_SAMPLES ) ;
    //Single Channel
    XAdcPs_SetSequencerMode (& XADC_Driver_Instance , XADCPS_SEQ_MODE_SINGCHAN ) ;
    XAdcPs_SetSingleChParams (& XADC_Driver_Instance , XADCPS_CH_AUX_MAX ,FALSE , FALSE , FALSE ) ;

}

unsigned short Goniometer_Read(void)
{
	// Set conversion sequence		 
//	unsigned short volt_raw = XAdcPs_GetAdcData(XADC_Driver_Ptr, XADCPS_CH_AUX_MAX);
	unsigned short volt_raw = XMedian_filter_Get_median_o(XMedian_filter_Ptr);
//	float volt_f = XSysMon_RawToExtVoltage(volt_raw);
	return volt_raw;
}
