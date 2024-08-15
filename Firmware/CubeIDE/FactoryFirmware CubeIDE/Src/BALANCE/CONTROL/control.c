#include "control.h"		
  /**************************************************************************
Author£ºMinibalance
Our aliexpress£ºhttps://minibalance.aliexpress.com
**************************************************************************/
int Balance_Pwm,Position_Pwm;
u8 Flag_Target,Position_Target;
/**************************************************************************
Function: all the control codes are in it.
5ms timing interrupt controlled by TIM1
**************************************************************************/
int TIM1_UP_IRQHandler(void)  
{    
	if(TIM1->SR&0X0001)//5ms timing interrupt
	{   
		  TIM1->SR&=~(1<<0);                                       //===Clear timer 1 interrupt flag bit                    
	     if(delay_flag==1)
			 {
				 if(++delay_50==10)	 delay_50=0,delay_flag=0;          //===Provide 50ms precise delay for main function.
			 }		
    	Encoder=Read_Encoder(4);             	                   //===Update encoder location information
      Angle_Balance=Get_Adc_Average(3,15);                     //===Postures	
     	Balance_Pwm =balance(Angle_Balance);                                          //===Angle PD control	
	    if(++Position_Target>4)	Position_Pwm=Position(Encoder),Position_Target=0;     //===Position PD control 25ms for one position control
      Moto=Balance_Pwm-Position_Pwm;        //===Calculate final motor PWM
		  Xianfu_Pwm();                         //===PWM limiting, anyway, the system instability caused by duty ratio 100%.
		  if(Turn_Off(Voltage)==0)              //===Low pressure and excessive inclination protection
			Set_Pwm(Moto);                        //===Assign to PWM register
	  	Led_Flash(100);                       //===LED flicker indicates normal operation of the system. 
	    Voltage=Get_battery_volt();           //===Get battery voltage	      
			Key();                                //===Scan button changes
	}       	
	 return 0;	  
} 

/**************************************************************************
Function: tilt PD control
Entry parameters: angle
Return value: tilt control PWM
**************************************************************************/
int balance(float Angle)
{  
   float Bias;                       //Inclination deviation
	 static float Last_Bias,D_Bias;    //PID dependent variables
	 int balance;                      //PWM return value 
	 Bias=Angle-ZHONGZHI;              //Find out the angle of equilibrium and mechanical correlation.
	 D_Bias=Bias-Last_Bias;            //Differential control is used to derive deviations.
	 balance=-Balance_KP*Bias-D_Bias*Balance_KD;   //===Motor PWM PD control for tilt angle control
   Last_Bias=Bias;                   //Keep the last deviation.
	 return balance;
}

/**************************************************************************
Function: position PD control
Entry parameters: encoder
Return value: position control PWM
**************************************************************************/
int Position(int Encoder)
{  
   static float Position_PWM,Last_Position,Position_Bias,Position_Differential;
	 static float Position_Least;
  	Position_Least =Encoder-Position_Zero;             //===
    Position_Bias *=0.8;		   
    Position_Bias += Position_Least*0.2;	             //===First order low pass filter 
	  Position_Differential=Position_Bias-Last_Position;
	  Last_Position=Position_Bias;
		Position_PWM=Position_Bias*Position_KP+Position_Differential*Position_KD; //= = speed control	
	  return Position_PWM;
}

/**************************************************************************
Function: assign to PWM register
Entry parameter: PWM
Return value: None
**************************************************************************/
void Set_Pwm(int moto)
{
    	if(moto<0)			AIN2=1,			AIN1=0;
			else 	          AIN2=0,			AIN1=1;
			PWMA=myabs(moto);
}

/**************************************************************************
Function: restrict PWM assignment
Entry parameters: None
Return value: None
**************************************************************************/
void Xianfu_Pwm(void)
{	
	  int Amplitude=6900;    //===PWM is full with a 7200 limit of 6900.
	  if(Moto<-Amplitude) Moto=-Amplitude;	
		if(Moto>Amplitude)  Moto=Amplitude;		
}
/**************************************************************************
Function: key to modify the running state of the car and control the position of the swing rod.
Entry parameters: None
Return value: None
**************************************************************************/
void Key(void)
{	
	int Position=1040; //The original position of the target position motor is 10000 turns a circle is 1040 and encoder precision is related, the default is that the motor rotates a circle, output 1040 jump edges
	static int tmp,flag,count;
	tmp=click_N_Double(100); 
	
	if(tmp==1)flag=1;//++
  if(tmp==2)flag=2;//--
	
	if(flag==1) //Clockwise movement of swing rod
	{
		Position_Zero+=4;
		count+=4;	
		if(count==Position) 	flag=0,count=0;
	}	
		if(flag==2) //Counter clockwise movement of swing rod
	{
		Position_Zero-=4;
		count+=4;	
		if(count==Position) 	flag=0,count=0;
	}
}

/**************************************************************************
Function of function: abnormal closing motor
Inlet parameters: voltage
Return value: 1: abnormal 0: normal
**************************************************************************/
u8 Turn_Off(int voltage)
{
	    u8 temp; 
	    static u8 count;
			if(1==Flag_Stop) //The battery voltage is too low, turn off the motor.
			{	      
      Flag_Stop=1;				
      temp=1;                                            
			AIN1=0;                                            
			AIN2=0;
      }
			else
      temp=0;
			
			
			if(Angle_Balance<(ZHONGZHI-800)||Angle_Balance>(ZHONGZHI+800)||voltage<700)count++;else count=0;
			if(count==120)
			{
				count=0;
				Flag_Stop=1;
			}	
      return temp;			
}
/**************************************************************************
Function: absolute value function
Entry parameters: int
Return Value: unsigned int
**************************************************************************/
int myabs(int a)
{ 		   
	  int temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}
