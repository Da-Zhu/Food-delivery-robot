#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "includes.h"
#include "key.h"
#include "lcd.h"
#include "ltdc.h" 
#include "sdram.h"
#include "w25qxx.h"
#include "nand.h"  
#include "mpu.h"
#include "sdmmc_sdcard.h"
#include "malloc.h"
#include "ff.h"
#include "exfuns.h"
#include "fontupd.h"
#include "text.h"
#include "ov5640.h" 
#include "dcmi.h"  
#include "pcf8574.h" 
#include "atk_qrdecode.h"



/************************************************
 ALIENTEK 阿波罗STM32F7开发板 UCOSIII实验
 例4-1 UCOSII移植实验
 
 UCOSIII中以下优先级用户程序不能使用，ALIENTEK
 将这些优先级分配给了UCOSIII的5个系统内部任务
 优先级0：中断服务服务管理任务 OS_IntQTask()
 优先级1：时钟节拍任务 OS_TickTask()
 优先级2：定时任务 OS_TmrTask()
 优先级OS_CFG_PRIO_MAX-2：统计任务 OS_StatTask()
 优先级OS_CFG_PRIO_MAX-1：空闲任务 OS_IdleTask()
 技术支持：www.openedv.com
 淘宝店铺：http://eboard.taobao.com 
 关注微信公众平台微信号："正点原子"，免费获取STM32资料。
 广州市星翼电子科技有限公司  
 作者：正点原子 @ALIENTEK
************************************************/

//任务优先级
#define START_TASK_PRIO		3
//任务堆栈大小	
#define START_STK_SIZE 		512
//任务控制块
OS_TCB StartTaskTCB;
//任务堆栈	
CPU_STK START_TASK_STK[START_STK_SIZE];
//任务函数
void start_task(void *p_arg);

//任务优先级
#define LED0_TASK_PRIO		4
//任务堆栈大小	
#define LED0_STK_SIZE 		512
//任务控制块
OS_TCB Led0TaskTCB;
//任务堆栈	
CPU_STK LED0_TASK_STK[LED0_STK_SIZE];
void led0_task(void *p_arg);

//任务优先级
#define LED1_TASK_PRIO		5
//任务堆栈大小	
#define LED1_STK_SIZE       128
//任务控制块
OS_TCB Led1TaskTCB;
//任务堆栈	
CPU_STK LED1_TASK_STK[LED1_STK_SIZE];
//任务函数
void led1_task(void *p_arg);

//任务优先级
#define FLOAT_TASK_PRIO		6
//任务堆栈大小
#define FLOAT_STK_SIZE		256
//任务控制块
OS_TCB	FloatTaskTCB;
//任务堆栈
__align(8) CPU_STK	FLOAT_TASK_STK[FLOAT_STK_SIZE];
//任务函数
void float_task(void *p_arg);



u16 qr_image_width;						//输入识别图像的宽度（长度=宽度）
u8 	readok=0;									//采集完一帧数据标识
u32 *dcmi_line_buf[2];				//摄像头采用一行一行读取,定义行缓存  
u16 *rgb_data_buf;						//RGB565帧缓存buf 
u16 dcmi_curline=0;						//摄像头输出数据,当前行编号	



//摄像头数据DMA接收完成中断回调函数
void qr_dcmi_rx_callback(void)
{  
	OS_ERR err;
	CPU_SR_ALLOC();
	u32 *pbuf;
	u16 i;
	pbuf=(u32*)(rgb_data_buf+dcmi_curline*qr_image_width);//将rgb_data_buf地址偏移赋值给pbuf
	
	if(DMADMCI_Handler.Instance->CR&(1<<19))//DMA使用buf1,读取buf0
	{ 
		for(i=0;i<qr_image_width/2;i++)
		{
			pbuf[i]=dcmi_line_buf[0][i];
		} 
	}else 										//DMA使用buf0,读取buf1
	{
		for(i=0;i<qr_image_width/2;i++)
		{
			pbuf[i]=dcmi_line_buf[1][i];
		} 
	} 
	dcmi_curline++;
}

//显示图像
void qr_show_image(u16 xoff,u16 yoff,u16 width,u16 height,u16 *imagebuf)
{
	OS_ERR err;
	CPU_SR_ALLOC();
	u16 linecnt=yoff;
	
	if(lcdltdc.pwidth!=0)//RGB屏
	{
		for(linecnt=0;linecnt<height;linecnt++)
		{
			LTDC_Color_Fill(xoff,linecnt+yoff,xoff+width-1,linecnt+yoff,imagebuf+linecnt*width);//RGB屏,DM2D填充 
		}
		
	}else LCD_Color_Fill(xoff,yoff,xoff+width-1,yoff+height-1,imagebuf);	//MCU屏,直接显示
}

//imagewidth:<=240;大于240时,是240的整数倍
//imagebuf:RGB图像数据缓冲区
void qr_decode(u16 imagewidth,u16 *imagebuf)
{
	OS_ERR err;
	CPU_SR_ALLOC();
	static u8 bartype=0; 
	u8 *bmp;
	u8 *result=NULL;
	u16 Color;
	u16 i,j;	
	u16 qr_img_width=0;						//输入识别器的图像宽度,最大不超过240!
	u8 qr_img_scale=0;						//压缩比例因子
	
	if(imagewidth>240)
	{
		if(imagewidth%240)return ;	//不是240的倍数,直接退出
		qr_img_width=240;
		qr_img_scale=imagewidth/qr_img_width;
	}else
	{
		qr_img_width=imagewidth;
		qr_img_scale=1;
	}  
	result=mymalloc(SRAMIN,1536);//申请识别结果存放内存
	bmp=mymalloc(SRAMDTCM,qr_img_width*qr_img_width);//DTCM管理内存为120K，这里申请240*240=56K 
	mymemset(bmp,0,qr_img_width*qr_img_width);
	if(lcdltdc.pwidth==0)//MCU屏,无需镜像
	{ 
		for(i=0;i<qr_img_width;i++)		
		{
			for(j=0;j<qr_img_width;j++)		//将RGB565图片转成灰度
			{	
				Color=*(imagebuf+((i*imagewidth)+j)*qr_img_scale); //按照qr_img_scale压缩成240*240
				*(bmp+i*qr_img_width+j)=(((Color&0xF800)>> 8)*76+((Color&0x7E0)>>3)*150+((Color&0x001F)<<3)*30)>>8;
			}		
		}
	}else	//RGB屏,需要镜像
	{
		for(i=0;i<qr_img_width;i++)		
		{
			for(j=0;j<qr_img_width;j++)		//将RGB565图片转成灰度
			{	
				Color=*(imagebuf+((i*imagewidth)+qr_img_width-j-1)*qr_img_scale);//按照qr_img_scale压缩成240*240
				*(bmp+i*qr_img_width+j)=(((Color&0xF800)>> 8)*76+((Color&0x7E0)>>3)*150+((Color&0x001F)<<3)*30)>>8;
			}		
		}		
	}
	atk_qr_decode(qr_img_width,qr_img_width,bmp,bartype,result);//识别灰度图片（注意：单次耗时约0.2S）
	
	if(result[0]==0)//没有识别出来
	{
		bartype++;
		if(bartype>=5)bartype=0; 
	}
	else if(result[0]!=0)//识别出来了，显示结果
	{	
		PCF8574_WriteBit(BEEP_IO,0);//打开蜂鸣器
		OSTimeDlyHMSM(0,0,0,100,OS_OPT_TIME_HMSM_STRICT,&err); //延时200ms
		PCF8574_WriteBit(BEEP_IO,1);
		POINT_COLOR=BLUE; 
		LCD_Fill(0,(lcddev.height+qr_image_width)/2+20,lcddev.width,lcddev.height,BLACK);
		Show_Str(0,(lcddev.height+qr_image_width)/2+20,lcddev.width,
								(lcddev.height-qr_image_width)/2-20,(u8*)result,16,0							
						);//LCD显示识别结果
		printf("\r\nresult:\r\n%s\r\n",result);//串口打印识别结果 		
	}
	myfree(SRAMDTCM,bmp);		//释放灰度图bmp内存
	myfree(SRAMIN,result);	//释放识别结果	
}  



int main(void)
{
    OS_ERR err;
	CPU_SR_ALLOC();
    
	float fac;
	
    Write_Through();                //透写
    Cache_Enable();                 //打开L1-Cache
    HAL_Init();				        //初始化HAL库
    Stm32_Clock_Init(432,25,2,9);   //设置时钟,216Mhz 
    delay_init(216);                //延时初始化
	uart_init(115200);		        //串口初始化
    LED_Init();                     //初始化LED
	KEY_Init();                     //初始化按键
	SDRAM_Init();                   //初始化SDRAM
	LCD_Init();                     //初始化LCD
	W25QXX_Init();				   				//初始化W25Q256
	PCF8574_Init();									//初始化PCF8574
	OV5640_Init();									//初始化OV5640
	my_mem_init(SRAMIN);            //初始化内部内存池
	my_mem_init(SRAMEX);            //初始化外部SDRAM内存池
	my_mem_init(SRAMDTCM);          //初始化内部DTCM内存池
	POINT_COLOR=RED; 
	LCD_Clear(BLACK); 	
	while(font_init()) 		//检查字库
	{	    
		LCD_ShowString(30,50,200,16,16,(u8*)"Font Error!");
		OSTimeDlyHMSM(0,0,0,200,OS_OPT_TIME_HMSM_STRICT,&err); //延时200ms				  
		LCD_Fill(30,50,240,66,WHITE);//清除显示	     
		OSTimeDlyHMSM(0,0,0,200,OS_OPT_TIME_HMSM_STRICT,&err); //延时200ms				  
	}  	 
 	Show_Str_Mid(0,0,(u8*)"阿波罗STM32F4/F7开发板",16,lcddev.width);	 			    	 
	Show_Str_Mid(0,20,(u8*)"二维码/条形码识别实验",16,lcddev.width);	
	while(OV5640_Init())//初始化OV5640
	{
		Show_Str(30,190,240,16,(u8*)"OV5640 错误!",16,0);
		OSTimeDlyHMSM(0,0,0,200,OS_OPT_TIME_HMSM_STRICT,&err); //延时200ms
	    LCD_Fill(30,190,239,206,WHITE);
		OSTimeDlyHMSM(0,0,0,200,OS_OPT_TIME_HMSM_STRICT,&err); //延时200ms
	}	
	//自动对焦初始化
	OV5640_RGB565_Mode();		//RGB565模式 
	OV5640_Focus_Init(); 
	OV5640_Light_Mode(0);		//自动模式
	OV5640_Color_Saturation(3);	//色彩饱和度0
	OV5640_Brightness(4);		//亮度0
	OV5640_Contrast(3);			//对比度0
	OV5640_Sharpness(33);		//自动锐度
	OV5640_Focus_Constant();//启动持续对焦
	DCMI_Init();						//DCMI配置 

		
	qr_image_width=lcddev.width;
	if(qr_image_width>480)qr_image_width=480;//这里qr_image_width设置为240的倍数
	if(qr_image_width==320)qr_image_width=240;
	Show_Str(0,(lcddev.height+qr_image_width)/2+4,240,16,(u8*)"识别结果：",16,1);
	
	dcmi_line_buf[0]=mymalloc(SRAMIN,qr_image_width*2);						//为行缓存接收申请内存	
	dcmi_line_buf[1]=mymalloc(SRAMIN,qr_image_width*2);						//为行缓存接收申请内存
	rgb_data_buf=mymalloc(SRAMEX,qr_image_width*qr_image_width*2);//为rgb帧缓存申请内存
	
	dcmi_rx_callback=qr_dcmi_rx_callback;//DMA数据接收中断回调函数
	DCMI_DMA_Init((u32)dcmi_line_buf[0],(u32)dcmi_line_buf[1],qr_image_width/2,DMA_MDATAALIGN_HALFWORD,DMA_MINC_ENABLE);//DCMI DMA配置  
	fac=800/qr_image_width;	//得到比例因子
	OV5640_OutSize_Set((1280-fac*qr_image_width)/2,(800-fac*qr_image_width)/2,qr_image_width,qr_image_width); 
	DCMI_Start(); 					//启动传输	 
		
	printf("SRAM IN:%d\r\n",my_mem_perused(SRAMIN));
	printf("SRAM EX:%d\r\n",my_mem_perused(SRAMEX));
	printf("SRAM DCTM:%d\r\n",my_mem_perused(SRAMDTCM)); 
	
	atk_qr_init();//初始化识别库，为算法申请内存
	
	printf("1SRAM IN:%d\r\n",my_mem_perused(SRAMIN));
	printf("1SRAM EX:%d\r\n",my_mem_perused(SRAMEX));
	printf("1SRAM DCTM:%d\r\n",my_mem_perused(SRAMDTCM));
	
	

	OSInit(&err);		            //初始化UCOSIII
	OS_CRITICAL_ENTER();            //进入临界区
	//创建开始任务
	OSTaskCreate((OS_TCB 	* )&StartTaskTCB,		//任务控制块
				 (CPU_CHAR	* )"start task", 		//任务名字
                 (OS_TASK_PTR )start_task, 			//任务函数
                 (void		* )0,					//传递给任务函数的参数
                 (OS_PRIO	  )START_TASK_PRIO,     //任务优先级
                 (CPU_STK   * )&START_TASK_STK[0],	//任务堆栈基地址
                 (CPU_STK_SIZE)START_STK_SIZE/10,	//任务堆栈深度限位
                 (CPU_STK_SIZE)START_STK_SIZE,		//任务堆栈大小
                 (OS_MSG_QTY  )0,					//任务内部消息队列能够接收的最大消息数目,为0时禁止接收消息
                 (OS_TICK	  )0,					//当使能时间片轮转时的时间片长度，为0时为默认长度，
                 (void   	* )0,					//用户补充的存储区
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR|OS_OPT_TASK_SAVE_FP, //任务选项,为了保险起见，所有任务都保存浮点寄存器的值
                 (OS_ERR 	* )&err);				//存放该函数错误时的返回值
	OS_CRITICAL_EXIT();	//退出临界区	 
	OSStart(&err);      //开启UCOSIII
    while(1)
    {
	} 
}

//开始任务函数
void start_task(void *p_arg)
{
	OS_ERR err;
	CPU_SR_ALLOC();
	p_arg = p_arg;

	CPU_Init();
#if OS_CFG_STAT_TASK_EN > 0u
   OSStatTaskCPUUsageInit(&err);  	//统计任务                
#endif
	
#ifdef CPU_CFG_INT_DIS_MEAS_EN		//如果使能了测量中断关闭时间
    CPU_IntDisMeasMaxCurReset();	
#endif

#if	OS_CFG_SCHED_ROUND_ROBIN_EN  //当使用时间片轮转的时候
	 //使能时间片轮转调度功能,设置默认的时间片长度s
	OSSchedRoundRobinCfg(DEF_ENABLED,10,&err);  
#endif		
	
	OS_CRITICAL_ENTER();	//进入临界区
	//创建LED0任务
	OSTaskCreate((OS_TCB 	* )&Led0TaskTCB,		
				 (CPU_CHAR	* )"led0 task", 		
                 (OS_TASK_PTR )led0_task, 			
                 (void		* )0,					
                 (OS_PRIO	  )LED0_TASK_PRIO,     
                 (CPU_STK   * )&LED0_TASK_STK[0],	
                 (CPU_STK_SIZE)LED0_STK_SIZE/10,	
                 (CPU_STK_SIZE)LED0_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,					
                 (void   	* )0,					
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR|OS_OPT_TASK_SAVE_FP,
                 (OS_ERR 	* )&err);				
				 
	//创建LED1任务
	OSTaskCreate((OS_TCB 	* )&Led1TaskTCB,		
				 (CPU_CHAR	* )"led1 task", 		
                 (OS_TASK_PTR )led1_task, 			
                 (void		* )0,					
                 (OS_PRIO	  )LED1_TASK_PRIO,     	
                 (CPU_STK   * )&LED1_TASK_STK[0],	
                 (CPU_STK_SIZE)LED1_STK_SIZE/10,	
                 (CPU_STK_SIZE)LED1_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,					
                 (void   	* )0,				
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR|OS_OPT_TASK_SAVE_FP, 
                 (OS_ERR 	* )&err);
				 
	//创建浮点测试任务
	OSTaskCreate((OS_TCB 	* )&FloatTaskTCB,		
				 (CPU_CHAR	* )"float test task", 		
                 (OS_TASK_PTR )float_task, 			
                 (void		* )0,					
                 (OS_PRIO	  )FLOAT_TASK_PRIO,     	
                 (CPU_STK   * )&FLOAT_TASK_STK[0],	
                 (CPU_STK_SIZE)FLOAT_STK_SIZE/10,	
                 (CPU_STK_SIZE)FLOAT_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,					
                 (void   	* )0,				
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR|OS_OPT_TASK_SAVE_FP, 
                 (OS_ERR 	* )&err);
	OS_CRITICAL_EXIT();	//进入临界区				 
	OS_TaskSuspend((OS_TCB*)&StartTaskTCB,&err);		//挂起开始任务			 
}

//led0任务函数
void led0_task(void *p_arg)
{
							 
 	u8 key;						   
	u8 i;
	
	OS_ERR err;
	p_arg = p_arg;
	

//		while(1)
//		{
//			key=KEY_Scan(0);//不支持连按
//			if(key)
//			{ 
//				OV5640_Focus_Single();  //按KEY0、KEY1、KEYUP手动单次自动对焦
//				
//				if(key==KEY2_PRES)break;//按KEY2结束识别
//			} 
//			if(readok==1)			//采集到了一帧图像
//			{		
//				readok=0;
//				qr_show_image((lcddev.width-qr_image_width)/2,(lcddev.height-qr_image_width)/2,qr_image_width,qr_image_width,rgb_data_buf);
//				qr_decode(qr_image_width,rgb_data_buf);
//			}
//			i++;
//			if(i==20)//DS0闪烁.
//			{
//				i=0;
//				LED0_Toggle;
//			}
////			OSTimeDlyHMSM(0,0,0,20,OS_OPT_TIME_HMSM_STRICT,&err); //延时200ms
//		}
//		atk_qr_destroy();//释放算法内存
//		printf("3SRAM IN:%d\r\n",my_mem_perused(SRAMIN));
//		printf("3SRAM EX:%d\r\n",my_mem_perused(SRAMEX));
//		printf("3SRAM DCTM:%d\r\n",my_mem_perused(SRAMDTCM)); 
		while(1)
		{
			LED0_Toggle;
			OSTimeDlyHMSM(0,0,0,200,OS_OPT_TIME_HMSM_STRICT,&err); //延时200ms
			if(readok==1)			//采集到了一帧图像
			{		
				readok=0;
				qr_show_image((lcddev.width-qr_image_width)/2,(lcddev.height-qr_image_width)/2,qr_image_width,qr_image_width,rgb_data_buf);
				qr_decode(qr_image_width,rgb_data_buf);
			}
		}

}

//led1任务函数
void led1_task(void *p_arg)
{
	p_arg = p_arg;
	while(1)
	{
		printf("aaaa\r\n");
        delay_ms(500);//延时500ms
	}
}

//浮点测试任务
void float_task(void *p_arg)
{
	CPU_SR_ALLOC();
	static double double_num=0.00;
	while(1)
	{
		double_num+=0.01f;
		OS_CRITICAL_ENTER();	//进入临界区
		printf("double_num的值为: %.4f\r\n",double_num);
		OS_CRITICAL_EXIT();		//退出临界区
		delay_ms(1000);			//延时1000ms
	}
}

