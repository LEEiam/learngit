/*
*********************************************************************************************************
*
*	模块名称 : 主程序模块。
*	文件名称 : main.c
*	版    本 : V1.1
*	说    明 : 本期教程主要是为了配合第8章：GPIO教程而做。
*              实验目的：
*                1. 本实验主要是学习GPIO的基本功能。
*              实验内容：
*                1. 创建一个周期为100ms的软件定时器（SysTick实验有讲解）。
*			     2. 主程序中检测软件定时器时间是否到，时间到了翻转四个LED。
*              实验步骤：
*                1. 请看STM32-V5开发板用户手册
*              注意事项：
*                1. 本实验推荐使用串口软件SecureCRT，要不串口打印效果不整齐。此软件在
*                   V5开发板光盘里面有。
*                2. 务必将编辑器的缩进参数和TAB设置为4来阅读本文件，要不代码显示不整齐。
*                3. 对于初学者，这个例子只需了解LED的驱动和使用即可，关于串口和嘀嗒定时
*                   器会在后面的教程中和大家讲述。
*
*	修改记录 :
*		版本号  日期       作者            说明
*		V1.0    2013-11-20 Eric2013     首发
*		V1.1    2015-03-23 Eric2013     1. 升级固件库到V1.5.0
*                                       2. 升级BSP板级支持包  
*
*	Copyright (C), 2015-2020, 安富莱电子 www.armfly.com
*
*********************************************************************************************************
*/

#include "bsp.h"			/* 底层硬件驱动 */

#include "arm_math.h"

/* 定义例程名和例程发布日期 */
#define EXAMPLE_NAME	"V5-001a_GPIO实验_跑马灯"
#define EXAMPLE_DATE	"2015-03-23"
#define DEMO_VER		"1.1"

uint8_t adjust_state=0;
float32_t out; //PID运算输出值
uint32_t ADC_Converted_Value_total=0;  //10个ADC_Converted_Value相加后的和
float32_t ADC_Converted_average_Value;  //  ADC_Converted_Value_total/10 后的值
uint16_t ADC_Converted_Value[10];   //读ADC1->DR寄存器

/*位置式PID*/
struct _pid{
float32_t Set; // 定义设定值
float32_t Actual; // 定义实际值
float32_t err; // 定义偏差值
float32_t err_last; // 定义上一个偏差值
float32_t Kp,Ki,Kd; // 定义比例、积分、微分系数
float32_t output; // 定义电压值（控制执行器的变量）
float32_t integral; // 定义积分值
float32_t mark;
}pid = { 0,0,1,0,0,0,0,0,0,0};


float32_t PID_realize(float32_t speed)
 {
        pid.Set=speed;
        pid.err=pid.Set-pid.Actual;
        pid.integral+=pid.err;
        pid.output=pid.Kp*pid.err+pid.Ki*pid.integral*pid.mark+pid.Kd*(pid.err-pid.err_last);
        pid.err_last=pid.err;
return pid.output;
}
 

/* 仅允许本文件内调用的函数声明 */
static void PrintfLogo(void);

/*
*********************************************************************************************************
*	函 数 名: main
*	功能说明: c程序入口
*	形    参：无
*	返 回 值: 错误代码(无需处理)
*********************************************************************************************************
*/
int main(void)
{
    float a;
	/*
		ST固件库中的启动文件已经执行了 SystemInit() 函数，该函数在 system_stm32f4xx.c 文件，主要功能是
	配置CPU系统的时钟，内部Flash访问时序，配置FSMC用于外部SRAM
	*/
    uint8_t i;
//    uint16_t ADC_Converted_Value_total=0;
//    float32_t ADC_Converted_average_Value;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
	bsp_Init();		/* 硬件初始化 ADC1还未开始软启动*/
    
	PrintfLogo();	/* 打印例程信息到串口1 */
    
    pid.Kp =150;  //比例因子
	pid.Ki =0.5;  //积分因子
	pid.Kd = 0;  //微分因子

	bsp_StartAutoTimer(0, 200);	/* 启动1个500ms的自动重装的定时器 */
    bsp_StartAutoTimer(1,300);

	/* 进入主程序循环体 */
	while (1)
	{
           if (bsp_CheckTimer(0))   //200ms打印一次PID参数
       {
//          printf("ADC_GetConversionValue=%d\n\r",ADC_GetConversionValue(ADC2));
        a= Current_Convert_Value(5);
       }
//        if(adjust_state)
//        {
//            TIM_ITConfig(TIM1,TIM_IT_Update,DISABLE);
//           adjust_state=0;//复位状态
//            ADC_Converted_Value_total=0;
//            for(i=0;i<10;i++)
//            {
//                
//                if( ( abs(ADC_Converted_Value[i]-ADC_Converted_Value[i-1]) >= 50) && i>=1)
//                {
//                    ADC_Converted_Value[i]=ADC_Converted_Value[i-1];
//                }
//                
//                ADC_Converted_Value_total=ADC_Converted_Value_total+ADC_Converted_Value[i];
//            }
//            
//            ADC_Converted_average_Value=(float32_t)ADC_Converted_Value_total/10.0;
//            
//           
//            pid.Actual=ADC_Converted_average_Value*3.0/4095;
//            
//            /*积分分离*/
//            if(abs(pid.err)>0.2)
//            {   
//                pid.mark=0.0;
//            }
//            else
//                pid.mark=1.0;
//            
//             out = PID_realize(1.453);//PID运算
//            
//            /*限幅*/
//            if(out>=(SystemCoreClock/40000-1)*0.65)
//            {
//                    out=(SystemCoreClock/40000-1)*0.65;
//  
//            }
//            if(out<(SystemCoreClock/40000-1)*0.35)
//            {
//                out=(SystemCoreClock/40000-1)*0.35;
//            }
//            TIM_ITConfig(TIM1,TIM_IT_Update,ENABLE);
//            
//        }
//       if (bsp_CheckTimer(0))   //200ms打印一次PID参数
//       {
//           printf("\r\n out = %f   pid.Actual = %f   pid.err= %f  ADC_Converted_average_Value=%f pid.integral=%f\n\r", out,   pid.Actual,pid.err,ADC_Converted_average_Value,pid.integral);
//       }
//        
////       if (bsp_CheckTimer(1))  //3秒打印1次nADC_Converted_Value
////       {      
////           for(i=0;i<10;i++)
////           {
//////                TIM_ITConfig(TIM1,TIM_IT_Update,DISABLE);
////               if(( abs(ADC_Converted_Value[i]-ADC_Converted_Value[i-1]) >= 100) && i>=1)
////               {
////                   printf("                  ");
////               }
////               printf("\r\nADC_Converted_Value[%d]= %d\n\r",i,ADC_Converted_Value[i]);
////            
////           }
//////            TIM_ITConfig(TIM1,TIM_IT_Update,ENABLE);
////        
////       }
           
    }
}
    

void TIM1_UP_TIM10_IRQHandler(void)
{
     uint32_t flag=0;
    if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET)//检查指定的TIM中断发生与否:TIM 中断源   
    {  
       
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);//清除TIMx的中断待处理位:TIM 中断源
      TIM_SetCompare1(TIM1,out);
        adjust_state=1;
 
        
       
        

         while(flag<=9)
        {
            
            ADC_SoftwareStartConv(ADC1);//ADC1转换开始  ADC_CR2_SWSTART写1
            if(ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC))
            {
                ADC_Converted_Value[flag]=ADC_GetConversionValue(ADC1);
                ++flag;
            }
        }



     
    }
    
}
/*
*********************************************************************************************************
*	函 数 名: PrintfLogo
*	功能说明: 打印例程名称和例程发布日期, 接上串口线后，打开PC机的超级终端软件可以观察结果
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
static void PrintfLogo(void)
{
	/* 检测CPU ID */
	{
		/* 参考手册：
			32.6.1 MCU device ID code
			33.1 Unique device ID register (96 bits)
		*/
		uint32_t CPU_Sn0, CPU_Sn1, CPU_Sn2;

		CPU_Sn0 = *(__IO uint32_t*)(0x1FFF7A10);
		CPU_Sn1 = *(__IO uint32_t*)(0x1FFF7A10 + 4);
		CPU_Sn2 = *(__IO uint32_t*)(0x1FFF7A10 + 8);

		printf("\r\nCPU : STM32F407IGT6, LQFP176, UID = %08X %08X %08X\n\r"
			, CPU_Sn2, CPU_Sn1, CPU_Sn0);
	}

	printf("\n\r");
	printf("*************************************************************\n\r");
	printf("* 例程名称   : %s\r\n", EXAMPLE_NAME);	/* 打印例程名称 */
	printf("* 例程版本   : %s\r\n", DEMO_VER);		/* 打印例程版本 */
	printf("* 发布日期   : %s\r\n", EXAMPLE_DATE);	/* 打印例程日期 */

	/* 打印ST固件库版本，这3个定义宏在stm32f10x.h文件中 */
	printf("* 固件库版本 : V%d.%d.%d (STM32F4xx_StdPeriph_Driver)\r\n", __STM32F4XX_STDPERIPH_VERSION_MAIN,
			__STM32F4XX_STDPERIPH_VERSION_SUB1,__STM32F4XX_STDPERIPH_VERSION_SUB2);
	printf("* \r\n");	/* 打印一行空格 */
	printf("* QQ    : 1295744630 \r\n");
	printf("* 旺旺  : armfly\r\n");
	printf("* Email : armfly@qq.com \r\n");
	printf("* 淘宝店: armfly.taobao.com\r\n");
	printf("* Copyright www.armfly.com 安富莱电子\r\n");
	printf("*************************************************************\n\r");
}

/***************************** 安富莱电子 www.armfly.com (END OF FILE) *********************************/
