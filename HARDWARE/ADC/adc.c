#include "adc.h"
#include "remoter.h"
#include "delay.h"

volatile u16 ADC1_RegConvertedVal[4];
/*
void Adc_Init(void)
{
    GPIO_InitTypeDef 		GPIO_InitStructure;
    ADC_InitTypeDef 		ADC_InitStructure;
    DMA_InitTypeDef			DMA_InitStructure;
    NVIC_InitTypeDef 		NVIC_InitStructure;

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);//GPIOC

    RCC_ADCCLKConfig(RCC_PCLK2_Div6);	//ADCCLK = PCLK2/6=72M/6=12MHz，ADC最大频率不能超过14MHz

    //电位器（油门）-> PC1检测端口设置
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    //电位器（刹车）-> PB1检测端口设置
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    //设置DMA1，用于自动存储ADC1和ADC2规则通道的转换值
    DMA_DeInit(DMA1_Channel1);
    DMA_StructInit(&DMA_InitStructure);
    DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;                 //ADC数据寄存器地址(源地址)
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)ADC1_RegConvertedVal;  	//转换值存储地址(目标地址)
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;                          //从外设到内存
    DMA_InitStructure.DMA_BufferSize = 4;                             			//传输大小
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;            //外设地址不增
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                     //内存地址自增
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; //外设数据单位(每次传输32位)
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;         //内存数据单位(每次传输32位)
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;                             //循环模式
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;                         //本DMA通道优先级(用了多个通道时，本参数才有效果)
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                                //没有使用内存到内存的传输
    DMA_Init(DMA1_Channel1, &DMA_InitStructure);

    DMA_ClearITPendingBit(DMA1_IT_TC1);                //清除通道1传输完成中断
    DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);    //打开通道1传输完成中断
    DMA_Cmd(DMA1_Channel1, ENABLE);                    //使能DMA1

    ADC_DeInit(ADC1);

    ADC_StructInit(&ADC_InitStructure);
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;  	//ADC1工作在独立模式
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;			//轮流采集各个通道的值
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;		//连续转换模式，触发后就会一直转换
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; //转换触发信号选择，使用一个软件信号触发ADC1
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;  //数据右对齐，ADC是12位，要存到DR寄存器的高16位或低16位，就有左右对齐问题，决定了高4位无效或低4位无效
    ADC_InitStructure.ADC_NbrOfChannel = 4;					//要进行ADC转换的通道数：电位器（油门）+电位器（刹车）
    ADC_Init(ADC1,&ADC_InitStructure);

    ADC_DMACmd(ADC1,ENABLE);								//使能ADC1的DMA

    ADC_RegularChannelConfig(ADC1,ADC_Channel_10,1,ADC_SampleTime_55Cycles5);	//48V电压测量
    ADC_RegularChannelConfig(ADC1,ADC_Channel_11,2,ADC_SampleTime_55Cycles5);	//备用1
    ADC_RegularChannelConfig(ADC1,ADC_Channel_12,3,ADC_SampleTime_55Cycles5);	//备用2
    ADC_RegularChannelConfig(ADC1,ADC_Channel_13,4,ADC_SampleTime_55Cycles5);	//备用3

    ADC_Cmd(ADC1, ENABLE);

    ADC_ResetCalibration(ADC1);                       //复位校准寄存器
    while(ADC_GetResetCalibrationStatus(ADC1));       //等待校准寄存器复位完成
    ADC_StartCalibration(ADC1);                       //ADC1开始校准
    while(ADC_GetCalibrationStatus(ADC1));            //等待校准完成

    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    ADC_SoftwareStartConvCmd(ADC1, ENABLE);    //给主ADC一个软件触发信号，之后ADC就会一直转换下去
}

float h_Voltage;
u16 h_ADC_PC1;
u16 h_ADC_PC2;
u16 h_ADC_PC3;

void DMA1_Channel1_IRQHandler(void)
{
    if(DMA_GetITStatus(DMA1_IT_TC1) != RESET)
    {
        DMA_ClearITPendingBit(DMA1_IT_TC1);    //清除DMA通道1传输完成中断

        h_Voltage = ADC1_RegConvertedVal[0]*3.3/4096;
        h_ADC_PC1 = ADC1_RegConvertedVal[1]*20/4096;
		h_ADC_PC1 = h_ADC_PC1*100+30000;
		
        h_ADC_PC2 = ADC1_RegConvertedVal[2]*100/4096;
        h_ADC_PC3 = ADC1_RegConvertedVal[3]*100/4096;
    }
}

*/
void  Adc_Init(void)
{ 	
	ADC_InitTypeDef ADC_InitStructure; 
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC |RCC_APB2Periph_ADC1	, ENABLE );	  //使能ADC1通道时钟
 

	RCC_ADCCLKConfig(RCC_PCLK2_Div6);   //设置ADC分频因子6 72M/6=12,ADC最大时间不能超过14M

	//PC3 作为模拟通道输入引脚                         
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//模拟输入引脚
	GPIO_Init(GPIOC, &GPIO_InitStructure);	

	ADC_DeInit(ADC1);  //复位ADC1 

	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	//ADC工作模式:ADC1和ADC2工作在独立模式
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;	//模数转换工作在单通道模式
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;	//模数转换工作在单次转换模式
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//转换由软件而不是外部触发启动
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	//ADC数据右对齐
	ADC_InitStructure.ADC_NbrOfChannel = 1;	//顺序进行规则转换的ADC通道的数目
	ADC_Init(ADC1, &ADC_InitStructure);	//根据ADC_InitStruct中指定的参数初始化外设ADCx的寄存器   

  
	ADC_Cmd(ADC1, ENABLE);	//使能指定的ADC1
	
	ADC_ResetCalibration(ADC1);	//使能复位校准  
	 
	while(ADC_GetResetCalibrationStatus(ADC1));	//等待复位校准结束
	
	ADC_StartCalibration(ADC1);	 //开启AD校准
 
	while(ADC_GetCalibrationStatus(ADC1));	 //等待校准结束
 
//	ADC_SoftwareStartConvCmd(ADC1, ENABLE);		//使能指定的ADC1的软件转换启动功能

}


//获得ADC值
//ch:通道值 0~3
u16 Get_Adc(u8 ch)   
{
  	//设置指定ADC的规则组通道，一个序列，采样时间
	ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_71Cycles5 );	//ADC1,ADC通道,采样时间为239.5周期	  			    
  
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);		//使能指定的ADC1的软件转换启动功能	
	 
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));//等待转换结束

	return ADC_GetConversionValue(ADC1);	//返回最近一次ADC1规则组的转换结果
}

u16 Get_Adc_Average(u8 ch,u8 times)
{
	u32 temp_val=0;
	u8 t;
	for(t=0;t<times;t++)
	{
		temp_val+=Get_Adc(ch);
		delay_us(300);
	}
	return temp_val/times;
} 	 

