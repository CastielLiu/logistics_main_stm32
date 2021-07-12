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

    RCC_ADCCLKConfig(RCC_PCLK2_Div6);	//ADCCLK = PCLK2/6=72M/6=12MHz��ADC���Ƶ�ʲ��ܳ���14MHz

    //��λ�������ţ�-> PC1���˿�����
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    //��λ����ɲ����-> PB1���˿�����
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    //����DMA1�������Զ��洢ADC1��ADC2����ͨ����ת��ֵ
    DMA_DeInit(DMA1_Channel1);
    DMA_StructInit(&DMA_InitStructure);
    DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;                 //ADC���ݼĴ�����ַ(Դ��ַ)
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)ADC1_RegConvertedVal;  	//ת��ֵ�洢��ַ(Ŀ���ַ)
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;                          //�����赽�ڴ�
    DMA_InitStructure.DMA_BufferSize = 4;                             			//�����С
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;            //�����ַ����
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                     //�ڴ��ַ����
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; //�������ݵ�λ(ÿ�δ���32λ)
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;         //�ڴ����ݵ�λ(ÿ�δ���32λ)
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;                             //ѭ��ģʽ
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;                         //��DMAͨ�����ȼ�(���˶��ͨ��ʱ������������Ч��)
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                                //û��ʹ���ڴ浽�ڴ�Ĵ���
    DMA_Init(DMA1_Channel1, &DMA_InitStructure);

    DMA_ClearITPendingBit(DMA1_IT_TC1);                //���ͨ��1��������ж�
    DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);    //��ͨ��1��������ж�
    DMA_Cmd(DMA1_Channel1, ENABLE);                    //ʹ��DMA1

    ADC_DeInit(ADC1);

    ADC_StructInit(&ADC_InitStructure);
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;  	//ADC1�����ڶ���ģʽ
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;			//�����ɼ�����ͨ����ֵ
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;		//����ת��ģʽ��������ͻ�һֱת��
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; //ת�������ź�ѡ��ʹ��һ������źŴ���ADC1
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;  //�����Ҷ��룬ADC��12λ��Ҫ�浽DR�Ĵ����ĸ�16λ���16λ���������Ҷ������⣬�����˸�4λ��Ч���4λ��Ч
    ADC_InitStructure.ADC_NbrOfChannel = 4;					//Ҫ����ADCת����ͨ��������λ�������ţ�+��λ����ɲ����
    ADC_Init(ADC1,&ADC_InitStructure);

    ADC_DMACmd(ADC1,ENABLE);								//ʹ��ADC1��DMA

    ADC_RegularChannelConfig(ADC1,ADC_Channel_10,1,ADC_SampleTime_55Cycles5);	//48V��ѹ����
    ADC_RegularChannelConfig(ADC1,ADC_Channel_11,2,ADC_SampleTime_55Cycles5);	//����1
    ADC_RegularChannelConfig(ADC1,ADC_Channel_12,3,ADC_SampleTime_55Cycles5);	//����2
    ADC_RegularChannelConfig(ADC1,ADC_Channel_13,4,ADC_SampleTime_55Cycles5);	//����3

    ADC_Cmd(ADC1, ENABLE);

    ADC_ResetCalibration(ADC1);                       //��λУ׼�Ĵ���
    while(ADC_GetResetCalibrationStatus(ADC1));       //�ȴ�У׼�Ĵ�����λ���
    ADC_StartCalibration(ADC1);                       //ADC1��ʼУ׼
    while(ADC_GetCalibrationStatus(ADC1));            //�ȴ�У׼���

    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    ADC_SoftwareStartConvCmd(ADC1, ENABLE);    //����ADCһ����������źţ�֮��ADC�ͻ�һֱת����ȥ
}

float h_Voltage;
u16 h_ADC_PC1;
u16 h_ADC_PC2;
u16 h_ADC_PC3;

void DMA1_Channel1_IRQHandler(void)
{
    if(DMA_GetITStatus(DMA1_IT_TC1) != RESET)
    {
        DMA_ClearITPendingBit(DMA1_IT_TC1);    //���DMAͨ��1��������ж�

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

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC |RCC_APB2Periph_ADC1	, ENABLE );	  //ʹ��ADC1ͨ��ʱ��
 

	RCC_ADCCLKConfig(RCC_PCLK2_Div6);   //����ADC��Ƶ����6 72M/6=12,ADC���ʱ�䲻�ܳ���14M

	//PC3 ��Ϊģ��ͨ����������                         
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//ģ����������
	GPIO_Init(GPIOC, &GPIO_InitStructure);	

	ADC_DeInit(ADC1);  //��λADC1 

	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	//ADC����ģʽ:ADC1��ADC2�����ڶ���ģʽ
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;	//ģ��ת�������ڵ�ͨ��ģʽ
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;	//ģ��ת�������ڵ���ת��ģʽ
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//ת��������������ⲿ��������
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	//ADC�����Ҷ���
	ADC_InitStructure.ADC_NbrOfChannel = 1;	//˳����й���ת����ADCͨ������Ŀ
	ADC_Init(ADC1, &ADC_InitStructure);	//����ADC_InitStruct��ָ���Ĳ�����ʼ������ADCx�ļĴ���   

  
	ADC_Cmd(ADC1, ENABLE);	//ʹ��ָ����ADC1
	
	ADC_ResetCalibration(ADC1);	//ʹ�ܸ�λУ׼  
	 
	while(ADC_GetResetCalibrationStatus(ADC1));	//�ȴ���λУ׼����
	
	ADC_StartCalibration(ADC1);	 //����ADУ׼
 
	while(ADC_GetCalibrationStatus(ADC1));	 //�ȴ�У׼����
 
//	ADC_SoftwareStartConvCmd(ADC1, ENABLE);		//ʹ��ָ����ADC1�����ת����������

}


//���ADCֵ
//ch:ͨ��ֵ 0~3
u16 Get_Adc(u8 ch)   
{
  	//����ָ��ADC�Ĺ�����ͨ����һ�����У�����ʱ��
	ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_71Cycles5 );	//ADC1,ADCͨ��,����ʱ��Ϊ239.5����	  			    
  
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);		//ʹ��ָ����ADC1�����ת����������	
	 
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));//�ȴ�ת������

	return ADC_GetConversionValue(ADC1);	//�������һ��ADC1�������ת�����
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

