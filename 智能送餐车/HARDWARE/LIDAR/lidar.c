#include "lidar.h"
#include "lcd.h"
#include "led.h"
#include "math.h"
//#include "motor.h"
#include "delay.h"


u8 a,m,Data[150];

float AngleFSA,AngleLSA,Anglei[100],Distance[100],x[100],y[100]; //³õÊ¼½Ç,½áÊø½Ç,ÖÐ¼ä½Ç,¾àÀë

u8 i,LSN,b,n,c;


void AngleFSA_Cal(void)   //³õÊ¼½Ç¼ÆËã
{
	u32 AngleFSAL,AngleFSAH;
		
	  if(a==5)
		{
		AngleFSAL=Data[4];
		AngleFSAH=Data[5];
		AngleFSA=(((AngleFSAH*256)+AngleFSAL)>>1)*0.015625;
		}
}

void AngleLSA_Cal(void)   //½áÊø½Ç¼ÆËã
{
  u32 AngleLSAL,AngleLSAH;
	
	  if(a==7)
		{
    AngleLSAL=Data[6];
		AngleLSAH=Data[7];
		AngleLSA=(((AngleLSAH*256)+AngleLSAL)>>1)*0.015625;

		}
}
		
void Distance_Cal(void)   //¾àÀë¼ÆËã
{ 
	  u32 DistanceH,DistanceL;
	  
		if((a>9)&&(a%2==1))
		{
	      DistanceL=Data[a-1];
				DistanceH=Data[a];
				Distance[b]=((DistanceH*256)+DistanceL)*0.25;
        b++;
		}
		

}

void Anglei_Cal(void)   //ÖÐ¼ä½Ç¼ÆËã
{ 
	if(a==3)
	{
		LSN=Data[3];
	}
	
	if(LSN==1)
	{
	   Anglei[n]=AngleFSA;
	}
	
	if(LSN>1)
	{		
			if((a>7)&&(i<=LSN))
			{
								Anglei[n]=((AngleLSA-AngleFSA)/(LSN-1))*(i-1)+AngleFSA;
								n++;
								i++;
			}
  }
	
}

void Adumbrate(void)   //»­ÂÖÀª
{
  float val;
	
	val=PI/180;
	x[c]=sin((360-Anglei[c])*val)*Distance[c];
	y[c]=cos((360-Anglei[c])*val)*Distance[c];
  c++;
				
}	

void evade(void)
{
  float Get_Ang,Get_Dis;
	int h;
	
	if(a==0)
	{
	h=0;
	}
	
//	move_forward();
  if(a>9)
	{
			Get_Ang=Anglei[h];
			Get_Dis=Distance[h];

			
			if((Get_Ang>0)&&(Get_Ang<30)&&(Get_Dis<400)&&(Get_Dis>0))
			{
//			  stop();
//				delay_ms(300);
//				move_backward();
//				delay_ms(300);
//				stop();
//				delay_ms(300);
//			  turn_left();
//				delay_ms(300);
//				move_forward();
			}
			else if((Get_Ang>330)&&(Get_Ang<360)&&(Get_Dis<400)&&(Get_Dis>0))
			{
//			  stop();
//				delay_ms(300);
//				move_backward();
//				delay_ms(300);
//				stop();
//				delay_ms(300);
//			  turn_right();
//				delay_ms(300);
//				move_forward();
			}
			else if((Get_Ang>40)&&(Get_Ang<70)&&(Get_Dis<400)&&(Get_Dis>0))
			{
//				turn_left();
//				delay_ms(300);
//				move_forward();

			}
			
			else if((Get_Ang<320)&&(Get_Ang>290)&&(Get_Dis<400)&&(Get_Dis>0))
			{
//				turn_right();
//				delay_ms(300);
//				move_forward();

			}
			else 
			{
//			  move_forward();	  
			}	
			
			h++;
  }
	
		
//	if((Get_Ang>0)&&(Get_Ang<30)&&(Get_Dis>200))
//	{
//	  move_forward();
//	}
//	if((Get_Ang<360)&&(Get_Ang>330)&&(Get_Dis>200))
//	{
//	  move_forward();
//	}
	
}


