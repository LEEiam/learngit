/*
*********************************************************************************************************
*
*	ģ������ : ������ģ�顣
*	�ļ����� : main.c
*	��    �� : V1.1
*	˵    �� : ���ڽ̳���Ҫ��Ϊ����ϵ�8�£�GPIO�̶̳�����
*              ʵ��Ŀ�ģ�
*                1. ��ʵ����Ҫ��ѧϰGPIO�Ļ������ܡ�
*              ʵ�����ݣ�
*                1. ����һ������Ϊ100ms�������ʱ����SysTickʵ���н��⣩��
*			     2. �������м�������ʱ��ʱ���Ƿ񵽣�ʱ�䵽�˷�ת�ĸ�LED��
*              ʵ�鲽�裺
*                1. �뿴STM32-V5�������û��ֲ�
*              ע�����
*                1. ��ʵ���Ƽ�ʹ�ô������SecureCRT��Ҫ�����ڴ�ӡЧ�������롣�������
*                   V5��������������С�
*                2. ��ؽ��༭��������������TAB����Ϊ4���Ķ����ļ���Ҫ��������ʾ�����롣
*                3. ���ڳ�ѧ�ߣ��������ֻ���˽�LED��������ʹ�ü��ɣ����ڴ��ں���શ�ʱ
*                   �����ں���Ľ̳��кʹ�ҽ�����
*
*	�޸ļ�¼ :
*		�汾��  ����       ����            ˵��
*		V1.0    2013-11-20 Eric2013     �׷�
*		V1.1    2015-03-23 Eric2013     1. �����̼��⵽V1.5.0
*                                       2. ����BSP�弶֧�ְ�  
*
*	Copyright (C), 2015-2020, ���������� www.armfly.com
*
*********************************************************************************************************
*/

#include "bsp.h"			/* �ײ�Ӳ������ */

#include "arm_math.h"

/* ���������������̷������� */
#define EXAMPLE_NAME	"V5-001a_GPIOʵ��_�����"
#define EXAMPLE_DATE	"2015-03-23"
#define DEMO_VER		"1.1"

uint8_t adjust_state=0;
float32_t out; //PID�������ֵ
uint32_t ADC_Converted_Value_total=0;  //10��ADC_Converted_Value��Ӻ�ĺ�
float32_t ADC_Converted_average_Value;  //  ADC_Converted_Value_total/10 ���ֵ
uint16_t ADC_Converted_Value[10];   //��ADC1->DR�Ĵ���

/*λ��ʽPID*/
struct _pid{
float32_t Set; // �����趨ֵ
float32_t Actual; // ����ʵ��ֵ
float32_t err; // ����ƫ��ֵ
float32_t err_last; // ������һ��ƫ��ֵ
float32_t Kp,Ki,Kd; // ������������֡�΢��ϵ��
float32_t output; // �����ѹֵ������ִ�����ı�����
float32_t integral; // �������ֵ
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
 

/* �������ļ��ڵ��õĺ������� */
static void PrintfLogo(void);

/*
*********************************************************************************************************
*	�� �� ��: main
*	����˵��: c�������
*	��    �Σ���
*	�� �� ֵ: �������(���账��)
*********************************************************************************************************
*/
int main(void)
{
    float a;
	/*
		ST�̼����е������ļ��Ѿ�ִ���� SystemInit() �������ú����� system_stm32f4xx.c �ļ�����Ҫ������
	����CPUϵͳ��ʱ�ӣ��ڲ�Flash����ʱ������FSMC�����ⲿSRAM
	*/
    uint8_t i;
//    uint16_t ADC_Converted_Value_total=0;
//    float32_t ADC_Converted_average_Value;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//����ϵͳ�ж����ȼ�����2
	bsp_Init();		/* Ӳ����ʼ�� ADC1��δ��ʼ������*/
    
	PrintfLogo();	/* ��ӡ������Ϣ������1 */
    
    pid.Kp =150;  //��������
	pid.Ki =0.5;  //��������
	pid.Kd = 0;  //΢������

	bsp_StartAutoTimer(0, 200);	/* ����1��500ms���Զ���װ�Ķ�ʱ�� */
    bsp_StartAutoTimer(1,300);

	/* ����������ѭ���� */
	while (1)
	{
           if (bsp_CheckTimer(0))   //200ms��ӡһ��PID����
       {
//          printf("ADC_GetConversionValue=%d\n\r",ADC_GetConversionValue(ADC2));
        a= Current_Convert_Value(5);
       }
//        if(adjust_state)
//        {
//            TIM_ITConfig(TIM1,TIM_IT_Update,DISABLE);
//           adjust_state=0;//��λ״̬
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
//            /*���ַ���*/
//            if(abs(pid.err)>0.2)
//            {   
//                pid.mark=0.0;
//            }
//            else
//                pid.mark=1.0;
//            
//             out = PID_realize(1.453);//PID����
//            
//            /*�޷�*/
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
//       if (bsp_CheckTimer(0))   //200ms��ӡһ��PID����
//       {
//           printf("\r\n out = %f   pid.Actual = %f   pid.err= %f  ADC_Converted_average_Value=%f pid.integral=%f\n\r", out,   pid.Actual,pid.err,ADC_Converted_average_Value,pid.integral);
//       }
//        
////       if (bsp_CheckTimer(1))  //3���ӡ1��nADC_Converted_Value
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
    if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET)//���ָ����TIM�жϷ������:TIM �ж�Դ   
    {  
       
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);//���TIMx���жϴ�����λ:TIM �ж�Դ
      TIM_SetCompare1(TIM1,out);
        adjust_state=1;
 
        
       
        

         while(flag<=9)
        {
            
            ADC_SoftwareStartConv(ADC1);//ADC1ת����ʼ  ADC_CR2_SWSTARTд1
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
*	�� �� ��: PrintfLogo
*	����˵��: ��ӡ�������ƺ����̷�������, ���ϴ����ߺ󣬴�PC���ĳ����ն�������Թ۲���
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void PrintfLogo(void)
{
	/* ���CPU ID */
	{
		/* �ο��ֲ᣺
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
	printf("* ��������   : %s\r\n", EXAMPLE_NAME);	/* ��ӡ�������� */
	printf("* ���̰汾   : %s\r\n", DEMO_VER);		/* ��ӡ���̰汾 */
	printf("* ��������   : %s\r\n", EXAMPLE_DATE);	/* ��ӡ�������� */

	/* ��ӡST�̼���汾����3���������stm32f10x.h�ļ��� */
	printf("* �̼���汾 : V%d.%d.%d (STM32F4xx_StdPeriph_Driver)\r\n", __STM32F4XX_STDPERIPH_VERSION_MAIN,
			__STM32F4XX_STDPERIPH_VERSION_SUB1,__STM32F4XX_STDPERIPH_VERSION_SUB2);
	printf("* \r\n");	/* ��ӡһ�пո� */
	printf("* QQ    : 1295744630 \r\n");
	printf("* ����  : armfly\r\n");
	printf("* Email : armfly@qq.com \r\n");
	printf("* �Ա���: armfly.taobao.com\r\n");
	printf("* Copyright www.armfly.com ����������\r\n");
	printf("*************************************************************\n\r");
}

/***************************** ���������� www.armfly.com (END OF FILE) *********************************/
