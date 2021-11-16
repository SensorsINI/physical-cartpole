#include "show.h"
#include "DataScope_DP.h"

  /**************************************************************************
Author��Minibalance
Our aliexpress��https://minibalance.aliexpress.com
**************************************************************************/
unsigned char i,temp;          //Count variable
unsigned char Send_Count; //Serial port needs to send the number of data.
float Vol;
/**************************************************************************
Function: OLED display
Entry parameters: none
Return Value: None
**************************************************************************/
void oled_show(void)
{
		//=============The first row shows the angle PD to control the P parameter Position_KP=======================//	
		                      OLED_ShowString(00,00,"B-KP");                   
		                      OLED_ShowNumber(40,00,Balance_KP,3,12);
	                        OLED_ShowString(57,00,"."),  
	                        OLED_ShowNumber(61,00,(int)(Balance_KP*10)%10,1,12);
	
	                        OLED_ShowString(95,00,"A:");	  
	                        OLED_ShowNumber(108,00,Amplitude1,2,12);
		//=============The second row shows the angle PD to control the D parameter.=======================//	
		                      OLED_ShowString(00,10,"B-KD");
		                      OLED_ShowNumber(40,10,Balance_KD,3,12);
	                        OLED_ShowString(57,10,"."),  
	                        OLED_ShowNumber(61,10,(int)(Balance_KD*10)%10,1,12);
	
	                        OLED_ShowString(95,10,"A:");	  
	                        OLED_ShowNumber(108,10,Amplitude2,2,12);
		//=============Third line display encoder Position_KP=======================//	
		                      OLED_ShowString(00,20,"P-KP");
		                      OLED_ShowNumber(40,20,Position_KP,3,12);
	                        OLED_ShowString(57,20,"."),  
	                        OLED_ShowNumber(61,20,(int)(Position_KP*10)%10,1,12);
													
												  OLED_ShowString(95,20,"A:");	  
	                        OLED_ShowNumber(108,20,Amplitude3,2,12);
		//=============Fourth line display encoder 1=======================//	
		                      OLED_ShowString(00,30,"P-KD");
		                      OLED_ShowNumber(40,30,Position_KD,3,12);
	                        OLED_ShowString(57,30,"."),  
	                        OLED_ShowNumber(61,30,(int)(Position_KD*10)%10,1,12);
													
													OLED_ShowString(95,30,"A:");	  
	                        OLED_ShowNumber(108,30,Amplitude4,2,12);
		//======This is the PD parameter to be adjusted for scroll menu selection.										
		  if(Menu==1)
	   	{
			 OLED_ShowChar(75,00,'Y',12,1);   
			 OLED_ShowChar(75,10,'N',12,1);
			 OLED_ShowChar(75,20,'N',12,1);
			 OLED_ShowChar(75,30,'N',12,1);
	  	}
		  else	if(Menu==2)
	   	{
			 OLED_ShowChar(75,00,'N',12,1);
			 OLED_ShowChar(75,10,'Y',12,1);
			 OLED_ShowChar(75,20,'N',12,1);
			 OLED_ShowChar(75,30,'N',12,1);
			}		
      else if(Menu==3)
	   	{			
			 OLED_ShowChar(75,00,'N',12,1);
			 OLED_ShowChar(75,10,'N',12,1);
			 OLED_ShowChar(75,20,'Y',12,1);
			 OLED_ShowChar(75,30,'N',12,1);
			}		
      else if(Menu==4)
	   	{				
			 OLED_ShowChar(75,00,'N',12,1);
			 OLED_ShowChar(75,10,'N',12,1);
			 OLED_ShowChar(75,20,'N',12,1);
			 OLED_ShowChar(75,30,'Y',12,1);
	 	  } 
	//=============Fifth line display voltage and target location=======================//			
			OLED_ShowString(80,40,"T:");	  
			OLED_ShowNumber(95,40,Position_Zero,5,12) ; 
			                    OLED_ShowString(00,40,"VOL:");
		                      OLED_ShowString(41,40,".");
		                      OLED_ShowString(63,40,"V");
		                      OLED_ShowNumber(28,40,Voltage/100,2,12);
		                      OLED_ShowNumber(51,40,Voltage%100,2,12);
		 if(Voltage%100<10) 	OLED_ShowNumber(45,40,0,2,12);
		//=============Sixth row display angular displacement sensor and encoder data=======================//
		OLED_ShowString(80,50,"P:");    OLED_ShowNumber(95,50,Encoder,5,12); 
		OLED_ShowString(0,50,"ADC:");  OLED_ShowNumber(30,50,Angle_Balance,4,12);
		//=============Refresh=======================//
		OLED_Refresh_Gram();	
	}

/**************************************************************************
Function: The virtual oscilloscope sends data to the host computer. Close the display.
Entry parameters: none
Return Value: None
**************************************************************************/
void DataScope(void)
{   
		DataScope_Get_Channel_Data( Angle_Balance, 1 );      
		DataScope_Get_Channel_Data( Encoder, 2 );         
		DataScope_Get_Channel_Data( 0, 3 );              
//		DataScope_Get_Channel_Data( 0 , 4 );   
//		DataScope_Get_Channel_Data(0, 5 ); //Just replace 0 with the data you want to display.
//		DataScope_Get_Channel_Data(0 , 6 );//Just replace 0 with the data you want to display.
//		DataScope_Get_Channel_Data(0, 7 );
//		DataScope_Get_Channel_Data( 0, 8 ); 
//		DataScope_Get_Channel_Data(0, 9 );  
//		DataScope_Get_Channel_Data( 0 , 10);
		Send_Count = DataScope_Data_Generate(3);
		for( i = 0 ; i < Send_Count; i++) 
		{
		while((USART1->SR&0X40)==0);  
		USART1->DR = DataScope_OutPut_Buffer[i]; 
		}
}

