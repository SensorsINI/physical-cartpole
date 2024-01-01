#include "angle_zynq.h"


XAdcPs XADC_Driver_Instance ;
XAdcPs *XADC_Driver_Ptr = &XADC_Driver_Instance;

void Goniometer_Init_Zynq(void)
{
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

unsigned short Goniometer_Read_Zynq(void)
{
	// Set conversion sequence		 
	unsigned short volt_raw = XAdcPs_GetAdcData (XADC_Driver_Ptr, XADCPS_CH_AUX_MAX);
//	float volt_f = XSysMon_RawToExtVoltage(volt_raw);
	return volt_raw/16;
}
