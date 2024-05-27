#include "oled.h"
#include "stdlib.h"
#include "oledfont.h"  	 
#include "delay.h"
 /**************************************************************************
Author£ºMinibalance
Our aliexpress£ºhttps://minibalance.aliexpress.com
**************************************************************************/		   
u8 OLED_GRAM[128][8];	 
void OLED_Refresh_Gram(void)
{
	u8 i,n;		    
	for(i=0;i<8;i++)  
	{  
		OLED_WR_Byte (0xb0+i,OLED_CMD);    //Setting page address (0~7)
		OLED_WR_Byte (0x00,OLED_CMD);      //Set display location - low address
		OLED_WR_Byte (0x10,OLED_CMD);      //Set display location - column high address
		for(n=0;n<128;n++)OLED_WR_Byte(OLED_GRAM[n][i],OLED_DATA); 
	}   
}

// write a byte to OLED.
//dat: to write data / commands
//cmd: data / command flag 0, indicating command; 1, representing data;
void OLED_WR_Byte(u8 dat,u8 cmd)
{	
	u8 i;			  
	if(cmd)
	  OLED_RS_Set();
	else 
	  OLED_RS_Clr();		  
	for(i=0;i<8;i++)
	{			  
		OLED_SCLK_Clr();
		if(dat&0x80)
		   OLED_SDIN_Set();
		else 
		   OLED_SDIN_Clr();
		OLED_SCLK_Set();
		dat<<=1;   
	}				 		  
	OLED_RS_Set();   	  
} 

	  	  
// turn on the OLED display. 
void OLED_Display_On(void)
{
	OLED_WR_Byte(0X8D,OLED_CMD);  //SET DCDC command
	OLED_WR_Byte(0X14,OLED_CMD);  //DCDC ON
	OLED_WR_Byte(0XAF,OLED_CMD);  //DISPLAY ON
}
//Close OLED display     
void OLED_Display_Off(void)
{
	OLED_WR_Byte(0X8D,OLED_CMD);  //SET DCDCcommand
	OLED_WR_Byte(0X10,OLED_CMD);  //DCDC OFF
	OLED_WR_Byte(0XAE,OLED_CMD);  //DISPLAY OFF
}		   			 
// clear screen function, clear the screen, the whole screen is black! And not lit!!!
void OLED_Clear(void)  
{  
	u8 i,n;  
	for(i=0;i<8;i++)for(n=0;n<128;n++)OLED_GRAM[n][i]=0X00;  
	OLED_Refresh_Gram();//update display
}
// draw a point
//x:0~127
//y:0~63
//t:1 fill 0, empty.			   
void OLED_DrawPoint(u8 x,u8 y,u8 t)
{
	u8 pos,bx,temp=0;
	if(x>127||y>63)return;//It's out of range.
	pos=7-y/8;
	bx=y%8;
	temp=1<<(7-bx);
	if(t)OLED_GRAM[x][pos]|=temp;
	else OLED_GRAM[x][pos]&=~temp;	    
}

// display a character at the specified location, including some characters.
//x:0~127
//y:0~63
//mode:0, reverse display; 1, normal display.
//size: choose font 16/12 
void OLED_ShowChar(u8 x,u8 y,u8 chr,u8 size,u8 mode)
{      			    
	u8 temp,t,t1;
	u8 y0=y;
	chr=chr-' ';// get the offset value.				   
    for(t=0;t<size;t++)
    {   
		if(size==12)temp=oled_asc2_1206[chr][t];  //Call 1206 font
		else temp=oled_asc2_1608[chr][t];		 //Call 1608 font 	                          
        for(t1=0;t1<8;t1++)
		{
			if(temp&0x80)OLED_DrawPoint(x,y,mode);
			else OLED_DrawPoint(x,y,!mode);
			temp<<=1;
			y++;
			if((y-y0)==size)
			{
				y=y0;
				x++;
				break;
			}
		}  	 
    }          
}
//m^n function
u32 oled_pow(u8 m,u8 n)
{
	u32 result=1;	 
	while(n--)result*=m;    
	return result;
}				  
// show 2 numbers.
//x, Y: starting coordinates
//len: number of digits
//size: font size
//mode: mode 0, fill mode; 1, overlay mode.
//num: value (0~4294967295);	  
void OLED_ShowNumber(u8 x,u8 y,u32 num,u8 len,u8 size)
{         	
	u8 t,temp;
	u8 enshow=0;						   
	for(t=0;t<len;t++)
	{
		temp=(num/oled_pow(10,len-t-1))%10;
		if(enshow==0&&t<(len-1))
		{
			if(temp==0)
			{
				OLED_ShowChar(x+(size/2)*t,y,' ',size,1);
				continue;
			}else enshow=1; 
		 	 
		}
	 	OLED_ShowChar(x+(size/2)*t,y,temp+'0',size,1); 
	}
} 
// display string
//x, y: starting coordinates
//*p: string start address
// 16 font.
void OLED_ShowString(u8 x,u8 y,const u8 *p)
{
#define MAX_CHAR_POSX 122
#define MAX_CHAR_POSY 58          
    while(*p!='\0')
    {       
        if(x>MAX_CHAR_POSX){x=0;y+=16;}
        if(y>MAX_CHAR_POSY){y=x=0;OLED_Clear();}
        OLED_ShowChar(x,y,*p,12,1);	 
        x+=8;
        p++;
    }  
}	   
//³õÊ¼»¯oled					    
void OLED_Init(void)
{ 	
   	 
 	RCC->APB2ENR|=1<<3;    // enable PORTB clock	   	 
	GPIOB->CRL&=0XFF000FFF; 
	GPIOB->CRL|=0X00222000;//PB3 4 5 push-pull
	 
 	RCC->APB2ENR|=1<<2;    //Enable PORTA clock  	 
	GPIOA->CRH&=0X0FFFFFFF; 
	GPIOA->CRH|=0X20000000;//PA15 push-pull  


	OLED_RST_Clr();
	delay_ms(100);
	OLED_RST_Set(); 
					  
	OLED_WR_Byte(0xAE,OLED_CMD); //Close display
	OLED_WR_Byte(0xD5,OLED_CMD); //Set clock frequency factor, oscillation frequency
	OLED_WR_Byte(80,OLED_CMD);   //[3:0], frequency divisor; [7:4], oscillation frequency.
	OLED_WR_Byte(0xA8,OLED_CMD); //Set the number of drivers
	OLED_WR_Byte(0X3F,OLED_CMD); //Default 0X3F (1/64) 
	OLED_WR_Byte(0xD3,OLED_CMD); //Set display offset
	OLED_WR_Byte(0X00,OLED_CMD); //The default is 0

	OLED_WR_Byte(0x40,OLED_CMD); //Set the start line [5:0] and the number of rows.
													    
	OLED_WR_Byte(0x8D,OLED_CMD); //Charge pump settings
	OLED_WR_Byte(0x14,OLED_CMD); //Bit2, on / off
	OLED_WR_Byte(0x20,OLED_CMD); //Set memory address mode
	OLED_WR_Byte(0x02,OLED_CMD); //[1:0], 00, column address mode; 01, row address mode; 10, page address mode; default 10;
	OLED_WR_Byte(0xA1,OLED_CMD); //Segment redefinition settings, bit0:0,0->0; 1,0->127;
	OLED_WR_Byte(0xC0,OLED_CMD); //Set COM scan direction; bit3:0, normal mode; 1, redefine mode COM[N-1]->COM0; N: drive path.
	OLED_WR_Byte(0xDA,OLED_CMD); //Setting up COM hardware pin configuration
	OLED_WR_Byte(0x12,OLED_CMD); //[5:4]To configure
		 
	OLED_WR_Byte(0x81,OLED_CMD); //Contrast settings
	OLED_WR_Byte(0xEF,OLED_CMD); //1~255; default 0X7F (brightness setting, larger and brighter)
	OLED_WR_Byte(0xD9,OLED_CMD); //Set up pre charge cycle
	OLED_WR_Byte(0xf1,OLED_CMD); //[3:0],PHASE 1;[7:4],PHASE 2;
	OLED_WR_Byte(0xDB,OLED_CMD); //Set VCOMH voltage multiplying rate
	OLED_WR_Byte(0x30,OLED_CMD); //[6:4] 000,0.65*vcc;001,0.77*vcc;011,0.83*vcc;

	OLED_WR_Byte(0xA4,OLED_CMD); //The global display is turned on; bit0:1, turn on; 0, turn off; (white screen / black screen).
	OLED_WR_Byte(0xA6,OLED_CMD); //Set the display mode; bit0:1, reverse display; 0, normal display.  						   
	OLED_WR_Byte(0xAF,OLED_CMD); //Open display	 
	OLED_Clear();
}  




