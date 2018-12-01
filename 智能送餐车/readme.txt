实验器材:
	阿波罗STM32F7开发板
	
实验目的:
	学习UCOSIII在STM32F7上的移植
	
硬件资源:
	1,DS0(连接在PB1)，DS1(连接在PB0上)
	2,串口1(波特率:115200,PA9/PA10连接在板载USB转串口芯片CH340上面) 	
	
实验现象:
	本实验在STM32F7开发板上移植了UCOSIII，版本为3.04.05。例程中创建了四个任务来测试移植是否成功。
	start_task任务创建其他3个测试任务，led0_task任务中LED0闪烁，led1_task任务中LED1闪烁，	
	float_task测试浮点运算。
	
注意事项:
	无

	 

					正点原子@ALIENTEK
					2016-6-26
					广州市星翼电子科技有限公司
					电话：020-38271790
					传真：020-36773971
					购买：http://shop62103354.taobao.com
					http://shop62057469.taobao.com
					公司网站：www.alientek.com
					技术论坛：www.openedv.com
