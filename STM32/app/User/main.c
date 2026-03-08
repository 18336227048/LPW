#include "system.h"
#include "SysTick.h"
#include "led.h"
#include "usart.h"
#include "key.h"

 //#define  ADC_DAN  1  //ADC单通道采集
//#define  ADC_DANDMA  2  //ADC单通道DMA采集
 //#define DAC_OUT  3    //DAC输出
// #define DAC_OUTDMA  4    //DAC输出DMA
 //#define  Soft_IIC  5    //软件IIC
 //#define  HARD_IIC  6    //硬件IIC
// #define IIC_DMA  7    //IIC单通道采集
 //#define  HARD_SPI  	 8    //SPI FLASH
 //#define  Soft_SPI  	 9    //软件SPI FLASH
 //#define USART_RECEIVE_DMA       10    //使用串口DMA接收不定长数据
//#define USART_SEND_DMA 11 // 使用串口DMA发送不定长数据
//#define IWDG_TEST 12 // 独立看门狗
//#define  WWDG_TEST 13 // 窗口看门狗
//#define  TIM_INT 14 // 定时器中断
//#define  TIM_PWM 15 // 定时器PWM
//#define TIM_SoftwarePWM 16 // 定时器软件PWM 
#define CAN_TEST 17 // CAN通信
//#define IAP_TEST 18 // IAP下载


#if defined(ADC_DAN)
/*******************************************************************************
 * 函数名         : ADCx_Init
 * 函数功能		   : ADC1单通道采样初始化（PA5/ADC1_IN5，软件触发、单次采样模式）
 * 输入参数       : 无
 * 输出参数       : 无
 * 注意事项       : 1. 仅初始化ADC1，采集引脚为PA5（ADC1_IN5）
 *                2. ADC时钟为21MHz（84MHz/4），符合≤36MHz的硬件限制
 *******************************************************************************/
void ADCx_Init(void)
{
	// 定义GPIO初始化结构体（配置ADC采集引脚）
	GPIO_InitTypeDef GPIO_InitStructure;
	// 定义ADC公共参数初始化结构体（所有ADC共享的全局配置）
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	// 定义ADC独立参数初始化结构体（仅针对ADC1的配置）
	ADC_InitTypeDef ADC_InitStructure;

	// 1. 使能GPIOA时钟（PA5为ADC采集引脚）
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	// 2. 使能ADC1时钟（ADC外设挂载在APB2总线上）
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	// 3. 配置PA5为模拟输入模式（ADC采集引脚必须配置为模拟输入）
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;	 // 模拟输入模式（禁用数字功能）
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;		 // 配置PA5引脚
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; // 浮空模式（模拟输入无需上下拉）
	GPIO_Init(GPIOA, &GPIO_InitStructure);			 // 应用GPIO配置到GPIOA端口

	// 可选：ADC1复位（清除残留配置，默认注释，需调试时开启）
	// RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,ENABLE);
	// RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,DISABLE);

	// 4. 配置ADC公共参数（所有ADC共享）
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent; // 独立模式（单ADC工作，不与ADC2/3协同）
															 // 两次采样阶段延迟（多ADC模式有效，单ADC时无实际作用，设为5个ADC时钟周期）
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled; // 禁用DMA（本代码为轮询模式）
																			// ADC预分频：4分频，ADC时钟=PCLK2(84MHz)/4=21MHz（≤36MHz，满足硬件要求）
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;
	ADC_CommonInit(&ADC_CommonInitStructure); // 应用ADC公共配置

	// 5. 配置ADC1独立参数
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b; // 12位分辨率（采样值范围：0~4095）
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;		   // 关闭扫描模式（仅采集单通道，无需扫描）duo
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;	   // 关闭连续转换（单次触发，单次采样）
														   // 禁用外部触发（使用软件触发ADC转换）
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; // 数据右对齐（常规用法，便于计算电压）
														   // 规则转换序列长度：1（仅采集1个通道）
	ADC_InitStructure.ADC_NbrOfConversion = 1;
	ADC_Init(ADC1, &ADC_InitStructure); // 应用ADC1配置

	ADC_Cmd(ADC1, ENABLE); // 使能ADC1（ADC初始化完成后必须使能才能工作）
}

/*******************************************************************************
 * 函数名         : Get_ADC_Value
 * 函数功能		   : 获取指定ADC通道的采样值，多次采样后取平均值（降低噪声）
 * 输入参数       : ch: ADC通道号（如ADC_Channel_5对应PA5）
 *                : times: 采样次数（建议取4/8/16次，提升精度）
 * 输出参数       : 多次采样的平均值（12位，范围0~4095）
 * 核心逻辑       : 1. 配置采样通道 → 2. 软件触发采样 → 3. 等待转换完成 → 4. 读取值累加 → 5. 取平均
 *******************************************************************************/
u16 Get_ADC_Value(u8 ch, u8 times)
{
	u32 temp_val = 0; // 累加采样值（用u32避免溢出）
	u8 t;			  // 循环计数器

	// 配置ADC1规则通道：指定采集通道、序列位置1、采样时间480个ADC时钟周期
	// 采样时间越长，精度越高（480个周期为最长采样时间，适合高阻抗传感器）
	ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_480Cycles);

	// 多次采样并累加值
	for (t = 0; t < times; t++)
	{
		ADC_SoftwareStartConv(ADC1); // 软件触发ADC1开始转换
									 // 等待转换完成标志（EOC）置1，未完成则循环等待
		while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC))
			;
		temp_val += ADC_GetConversionValue(ADC1); // 读取采样值并累加
		delay_ms(5);							  // 每次采样间隔5ms，避免数据抖动
	}

	return temp_val / times; // 返回多次采样的平均值
}
u8 USART1_TX_BUF[10];
int main()
{
	u8 i = 0;
	u16 ADC_Value; // ADC采样值
	float Voltage; // 电压值
	uint32_t int_data;
	float we = 125.36f;
	ADCx_Init();
	SysTick_Init(168);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); // 中断优先级分组 分2组
	LED_Init();
	USART1_Init(115200);

	while (1)
	{
		i++;
		if (i % 20 == 0)
		{
			LED1 = !LED1;
		}
		// 步骤1：定义32位整数，先拼接4个字节（强制转32位防止溢出）
		/*int_data = ((uint32_t)USART1_RX_BUF[0] << 24)   // 第0字节→32位整数的最高位
						 | ((uint32_t)USART1_RX_BUF[1] << 16)   // 第1字节→次高位
						 | ((uint32_t)USART1_RX_BUF[2] << 8)    // 第2字节→次低位
						 | (uint32_t)USART1_RX_BUF[3];          // 第3字节→最低位
						  temp  = *(float *)&int_data;*/

		// int_data = *(uint32_t *)&we;

		// 步骤2：按大端序拆分32位整数到字节数组（加&0xFF确保只取8位）
		/*USART1_TX_BUF[0] = (*(uint32_t *)&we >> 24) & 0xFF;  // 最高位字节
		USART1_TX_BUF[1] = (*(uint32_t *)&we >> 16) & 0xFF;  // 次高位字节
		USART1_TX_BUF[2] = (*(uint32_t *)&we >> 8)  & 0xFF;  // 次低位字节
		USART1_TX_BUF[3] =  (*(uint32_t *)&we)        & 0xFF;  // 最低位字节
 
		// 发送4个字节（函数参数匹配：缓冲区指针 + 长度）
		Usart1_SendNBytes(USART1_TX_BUF, 4);*/

	    ADC_Value = Get_ADC_Value(ADC_Channel_5, 20);  // 获取ADC1通道5的采样值（16次采样，提升精度）
		 Voltage = (float)ADC_Value * 3.3 / 4095;  // 计算电压值（ADC1范围0~4095，对应3.3V，此处做简单转换）
		 printf("ADC_Value=%d Voltage=%.2fV\n",ADC_Value,Voltage);  // 打印采样值到串口*/
		delay_ms(500); // 间隔500ms
	}
}

#elif defined(ADC_DANDMA)

// ************************ 新增宏定义 ************************
#define ADC_DMA_BUF_SIZE 10				  // DMA缓存数组大小（可根据需求调整，建议≥采样平均次数）
#define ADC_DEFAULT_CHANNEL ADC_Channel_5 // 默认采集PA5（ADC1_IN5）

// ************************ 全局变量 ************************
u16 ADC_DMA_Buf[ADC_DMA_BUF_SIZE]; // DMA接收缓存数组（ADC采样值自动存入此处）

/*******************************************************************************
 * 函数名         : ADC_DMA_Config
 * 函数功能		   : 配置ADC1对应的DMA通道（DMA2_Stream0, Channel0）
 * 输入参数       : 无
 * 输出参数       : 无
 * 注意事项       : 1. STM32F4的ADC1_RX固定对应DMA2_Stream0 + Channel0
 *                2. 配置为循环模式，缓存存满后自动从头覆盖
 *******************************************************************************/
void ADC_DMA_Config(void)
{
	DMA_InitTypeDef DMA_InitStructure;

	// 1. 使能DMA2时钟（DMA2挂载在AHB1总线上）
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

	// 2. 复位DMA2 Stream0（清除残留配置，避免干扰）
	DMA_DeInit(DMA2_Stream0);
	while (DMA_GetCmdStatus(DMA2_Stream0) != DISABLE)
		; // 等待复位完成

	// 3. 配置DMA核心参数
	DMA_InitStructure.DMA_Channel = DMA_Channel_0;								// ADC1_RX对应DMA通道0
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&ADC1->DR;					// 外设基地址：ADC1数据寄存器
	DMA_InitStructure.DMA_Memory0BaseAddr = (u32)ADC_DMA_Buf;					// 内存基地址：DMA缓存数组
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;						// 数据方向：外设→内存
	DMA_InitStructure.DMA_BufferSize = ADC_DMA_BUF_SIZE;						// 缓存数组长度
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;			// 外设地址不递增（固定ADC1->DR）
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;						// 内存地址递增（逐个存入数组）
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; // 外设数据宽度：16位（ADC12位结果）
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;			// 内存数据宽度：16位
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;								// 循环模式（持续采集，自动重装载）
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;							// 高优先级（避免数据丢失）
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;						// 禁用FIFO模式（直接传输）
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;			// FIFO阈值：半满（FIFO满时停止传输）
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;					// 内存单次突发传输
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;			// 外设单次突发传输

	// 4. 初始化并使能DMA
	DMA_Init(DMA2_Stream0, &DMA_InitStructure);
	DMA_Cmd(DMA2_Stream0, ENABLE);
}

/*******************************************************************************
 * 函数名         : ADCx_Init
 * 函数功能		   : ADC1+DMA初始化（PA5/ADC1_IN5，连续采样、DMA自动传输模式）
 * 输入参数       : 无
 * 输出参数       : 无
 * 注意事项       : 1. 仅初始化ADC1，采集引脚为PA5（ADC1_IN5）
 *                2. ADC时钟为21MHz（84MHz/4），符合≤36MHz的硬件限制
 *                3. 开启连续转换，ADC触发后自动循环采样，DMA后台传输
 *******************************************************************************/
void ADCDMA_Init(void)
{
	// 定义GPIO初始化结构体（配置ADC采集引脚）
	GPIO_InitTypeDef GPIO_InitStructure;
	// 定义ADC公共参数初始化结构体（所有ADC共享的全局配置）
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	// 定义ADC独立参数初始化结构体（仅针对ADC1的配置）
	ADC_InitTypeDef ADC_InitStructure;

	// 1. 使能GPIOA时钟（PA5为ADC采集引脚）
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	// 2. 使能ADC1时钟（ADC外设挂载在APB2总线上）
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	// 3. 配置PA5为模拟输入模式（ADC采集引脚必须配置为模拟输入）
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;	 // 模拟输入模式（禁用数字功能）
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;		 // 配置PA5引脚
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; // 浮空模式（模拟输入无需上下拉）
	GPIO_Init(GPIOA, &GPIO_InitStructure);			 // 应用GPIO配置到GPIOA端口

	// 可选：ADC1复位（清除残留配置，默认注释，需调试时开启）
	// RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,ENABLE);
	// RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,DISABLE);

	// 4. 配置ADC公共参数（所有ADC共享）
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent; // 独立模式（单ADC工作，不与ADC2/3协同）
															 // 两次采样阶段延迟（多ADC模式有效，单ADC时无实际作用，设为5个ADC时钟周期）
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled; // 单ADC禁用多ADC DMA模式
																			// ADC预分频：4分频，ADC时钟=PCLK2(84MHz)/4=21MHz（≤36MHz，满足硬件要求）
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;
	ADC_CommonInit(&ADC_CommonInitStructure); // 应用ADC公共配置

	// 5. 配置ADC1独立参数（核心修改：开启连续转换）
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b; // 12位分辨率（采样值范围：0~4095）
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;		   // 关闭扫描模式（仅采集单通道，无需扫描）
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;	   // 开启连续转换（自动循环采样，无需重复触发）
														   // 禁用外部触发（使用软件触发首次转换，后续自动循环）
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; // 数据右对齐（常规用法，便于计算电压）
														   // 规则转换序列长度：1（仅采集1个通道）
	ADC_InitStructure.ADC_NbrOfConversion = 1;
	ADC_Init(ADC1, &ADC_InitStructure); // 应用ADC1配置

	// 6. 配置ADC1规则通道（固定通道，无需每次采样配置）
	ADC_RegularChannelConfig(ADC1, ADC_DEFAULT_CHANNEL, 1, ADC_SampleTime_480Cycles);

	// 7. 初始化并使能DMA
	ADC_DMA_Config();

	// 8. 使能ADC1的DMA请求（ADC转换完成后自动触发DMA传输）
	ADC_DMACmd(ADC1, ENABLE);

	// 9. 使能ADC1（ADC初始化完成后必须使能才能工作）
	ADC_Cmd(ADC1, ENABLE);

	// 10. 软件触发首次转换（连续模式下触发后自动循环采样）
	ADC_SoftwareStartConv(ADC1);
}

/*******************************************************************************
 * 函数名         : Get_ADC_DMA_Value
 * 函数功能		   : 从DMA缓存中读取指定次数的采样值，取平均值（降低噪声）
 * 输入参数       : times: 采样次数（建议≤ADC_DMA_BUF_SIZE，超出则取最大值）
 * 输出参数       : 多次采样的平均值（12位，范围0~4095）
 * 核心逻辑       : 1. 从DMA缓存数组末尾读取最新的times个数据（避免读取旧数据）
 *                2. 累加后取平均，返回结果
 *******************************************************************************/
u16 Get_ADC_DMA_Value(u8 times)
{
	u32 temp_val = 0; // 累加采样值（用u32避免溢出）
	u8 t;			  // 循环计数器

	// 合法性检查：采样次数为0或超出缓存大小，默认取缓存全部数据
	if (times == 0 || times > ADC_DMA_BUF_SIZE)
	{
		times = ADC_DMA_BUF_SIZE;
	}

	// 从DMA缓存数组末尾读取最新的times个数据（循环模式下末尾为最新值）
	for (t = 0; t < times; t++)
	{

		temp_val += ADC_DMA_Buf[t]; // 累加采样值
	}

	return temp_val / times; // 返回多次采样的平均值
}

int main()
{
	u8 i = 0;

	u16 ADCDMA_Value;	  // ADC DMA采样值
	float ADCDMA_Voltage; // ADC DMA采样电压值
	ADCDMA_Init();

	SysTick_Init(168);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); // 中断优先级分组 分2组
	LED_Init();
	USART1_Init(115200);

	while (1)
	{
		i++;
		if (i % 20 == 0)
		{
			LED1 = !LED1;
		}
		ADCDMA_Value = Get_ADC_DMA_Value(10);
		// 转换为实际电压值（参考电压3.3V，12位分辨率：3.3/4096）
		ADCDMA_Voltage = ADCDMA_Value * (3.3f / 4096);

		// 打印结果（需自行实现串口打印函数）
		printf("ADC DMA Average Value: %d, Voltage: %.2fV\r\n", ADCDMA_Value, ADCDMA_Voltage);
		delay_ms(500); // 间隔500ms
	}
}
#elif defined(DAC_OUT)
/*******************************************************************************
 * 函数名         : DAC1_Init
 * 函数功能		   : DAC1通道1（PA4）初始化函数
 * 输入           : 无
 * 输出           : 无
 * 备注           : 配置PA4为模拟输入，DAC1无触发、无波形生成、关闭输出缓冲，初始值0
 *******************************************************************************/
void DAC1_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure; // GPIO初始化结构体
	DAC_InitTypeDef DAC_InitStructure;	 // DAC初始化结构体

	// 1. 使能GPIOA时钟（AHB1总线）
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	// 2. 使能DAC外设时钟（APB1总线）
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);

	// 3. 配置PA4为模拟输入模式（DAC输出引脚必须设为模拟输入）
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;		 // 选择PA4引脚（DAC1通道1）
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;	 // 模拟输入模式（禁用数字电路）
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; // 无上下拉（模拟引脚无需拉电阻）
	GPIO_Init(GPIOA, &GPIO_InitStructure);			 // 初始化GPIOA

	// 4. 配置DAC1通道1参数
	DAC_InitStructure.DAC_Trigger = DAC_Trigger_None;						  // 无触发源（软件手动更新输出）
	DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;			  // 不生成波形（普通电压输出）
	DAC_InitStructure.DAC_LFSRUnmask_TriangleAmplitude = DAC_LFSRUnmask_Bit0; // 波形参数（无波形时无效）
	DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Disable;			  // 关闭输出缓冲（减少延迟）
	DAC_Init(DAC_Channel_1, &DAC_InitStructure);							  // 初始化DAC1通道1

	// 5. 设置DAC1通道1初始值：12位右对齐，初始化数值0（对应0V）
	DAC_SetChannel1Data(DAC_Align_12b_R, 0);

	// 6. 使能DAC1通道1
	DAC_Cmd(DAC_Channel_1, ENABLE);
}

/*******************************************************************************
 * 函数名         : main
 * 函数功能		   : 主函数
 * 输入           : 无
 * 输出           : 无
 * 备注           : 按键UP增加DAC输出电压，按键KEY0减少电压，实时打印电压值，LED闪烁
 *******************************************************************************/
int main()
{
	u8 i = 0;		   // 循环计数变量
	u8 key;			   // 按键扫描结果
	int dac_value = 0; // DAC输出数值（0~4095）
	u16 dacval;		   // 读取DAC实际输出数值
	float dac_vol;	   // DAC输出电压值（V）

	// 系统初始化
	SysTick_Init(168);								// SysTick延时初始化（168MHz系统时钟）
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); // 中断分组：2组（2位抢占+2位响应）
	LED_Init();										// LED初始化（用于状态指示）
	USART1_Init(115200);							// USART1初始化（波特率115200，用于打印）
	KEY_Init();										// 按键初始化（UP/KEY0）
	DAC1_Init();									// DAC1初始化

	// 主循环
	while (1)
	{
		// 1. 按键扫描（参数0：不支持连按；参数1：支持连按）
		key = KEY_Scan(0);

		// 2. 按键UP按下：增加DAC输出数值（每次+400）
		if (key == KEY_UP_PRESS)
		{
			dac_value += 400;
			// 防止数值超过12位最大值4095
			if (dac_value >= 4000)
			{
				dac_value = 4095;
			}
			// 更新DAC输出数值（12位右对齐）
			DAC_SetChannel1Data(DAC_Align_12b_R, dac_value);
		}
		// 3. 按键KEY0按下：减少DAC输出数值（每次-400）
		else if (key == KEY0_PRESS)
		{
			dac_value -= 400;
			// 防止数值低于0
			if (dac_value <= 0)
			{
				dac_value = 0;
			}
			// 更新DAC输出数值
			DAC_SetChannel1Data(DAC_Align_12b_R, dac_value);
		}

		// 4. LED1每200ms闪烁一次（20*10ms），指示程序运行
		i++;
		if (i % 20 == 0)
		{
			LED1 = !LED1;
		}

		// 5. 每500ms打印一次DAC输出电压（50*10ms）
		if (i % 50 == 0)
		{
			// 读取DAC1通道1的实际输出数值（12位）
			dacval = DAC_GetDataOutputValue(DAC_Channel_1);
			// 计算输出电压：Vout = 数值 * (Vref+/4095)，Vref+=3.3V
			dac_vol = (float)dacval * (3.3 / 4095);
			// 串口打印电压值（保留2位小数）和数值
			printf("输出DAC电压值为：%.2fV DAC_VAL:%d\r\n", dac_vol, dacval);
		}

		// 延时10ms（降低扫描频率，减少CPU占用）
		delay_ms(10);
	}
}
#elif defined(DAC_OUTDMA)

// 全局变量：DAC输出数值数组（循环输出8个不同电压，可自定义）
u16 dac_tbuf[8] = {0x000, 0x400, 0x800, 0xC00, 0x1000, 0x1400, 0x1800, 0x1FFF};
// 全局变量：DMA接收缓冲区（用于接收DMA传输的DAC输出数值）
// 对应电压：0V、0.32V、0.64V、0.96V、1.28V、1.60V、1.92V、3.3V（3.3*8191/4095≈3.3V）

/*******************************************************************************
 * 函数名         : DAC1_DMA_Init
 * 函数功能		   : 初始化DAC1的DMA通道（DMA1_Stream5_Channel7）
 * 输入           : 无
 * 输出           : 无
 * 备注           : 配置DMA为循环模式，内存→外设，半字传输，内存地址递增
 *******************************************************************************/
void DAC1_DMA_Init(void)
{
	DMA_InitTypeDef DMA_InitStructure;

	// 1. 使能DMA1时钟（AHB1总线）
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

	// 2. 配置DMA1 Stream5（DAC1对应DMA1 Stream5 Channel7）
	DMA_DeInit(DMA1_Stream5); // 复位DMA1 Stream5
	while (DMA_GetCmdStatus(DMA1_Stream5) != DISABLE)
		; // 等待DMA失能

	DMA_InitStructure.DMA_Channel = DMA_Channel_7;								// 选择通道7（DAC1对应通道7）
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&DAC->DHR12R1;				// DAC1通道1 12位右对齐数据寄存器
	DMA_InitStructure.DMA_Memory0BaseAddr = (u32)dac_tbuf;						// 内存基地址（数值数组）
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;						// 传输方向：内存→外设
	DMA_InitStructure.DMA_BufferSize = 8;										// 缓冲区大小（数组元素个数）
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;			// 外设地址不递增（固定写DAC寄存器）
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;						// 内存地址递增（依次读取数组元素）
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; // 外设数据宽度：半字（16位）
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;			// 内存数据宽度：半字（16位）
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;								// 循环模式（传输完自动从头开始）
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;							// DMA优先级：高
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;						// 禁用FIFO模式
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single; // 单次突发传输
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

	DMA_Init(DMA1_Stream5, &DMA_InitStructure); // 初始化DMA
	DMA_Cmd(DMA1_Stream5, ENABLE);				// 使能DMA1 Stream5
}

/*******************************************************************************
 * 函数名         : TIM6_Init
 * 函数功能		   : 初始化TIM6定时器（用于触发DAC转换）
 * 输入           : arr - 自动重装值  psc - 预分频系数
 * 输出           : 无
 * 备注           : TIM6为基本定时器，仅支持更新事件触发，配置为100ms触发一次
 *******************************************************************************/
void TIM6_Init(u16 arr, u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

	// 1. 使能TIM6时钟（APB1总线）
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);

	// 2. 配置TIM6时基参数
	TIM_TimeBaseStructure.TIM_Period = arr;						// 自动重装值
	TIM_TimeBaseStructure.TIM_Prescaler = psc;					// 预分频系数
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;		// 时钟分频：不分频
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; // 向上计数
	TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);

	// 3. 选择TIM6的TRGO触发源：更新事件（计数到arr）
	TIM_SelectOutputTrigger(TIM6, TIM_TRGOSource_Update);
	// 4. 使能TIM6
	TIM_Cmd(TIM6, ENABLE);
}

/*******************************************************************************
 * 函数名         : DAC1_Init
 * 函数功能		   : DAC1通道1（PA4）初始化函数（DMA+TIM6触发版）
 * 输入           : 无
 * 输出           : 无
 * 备注           : 配置PA4为模拟输入，DAC1由TIM6_TRGO触发，开启DMA，关闭输出缓冲
 *******************************************************************************/
void DAC1_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	DAC_InitTypeDef DAC_InitStructure;

	// 1. 使能GPIOA和DAC时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);

	// 2. 配置PA4为模拟输入模式
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// 3. 初始化DAC1 DMA通道
	DAC1_DMA_Init();

	// 4. 配置DAC1通道1参数（TIM6触发+DMA）
	DAC_InitStructure.DAC_Trigger = DAC_Trigger_T6_TRGO;			// 触发源：TIM6_TRGO
	DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None; // 无波形生成
	DAC_InitStructure.DAC_LFSRUnmask_TriangleAmplitude = DAC_LFSRUnmask_Bit0;
	DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Disable; // 关闭输出缓冲
	DAC_Init(DAC_Channel_1, &DAC_InitStructure);

	// 5. 使能DAC1通道1的DMA请求
	DAC_DMACmd(DAC_Channel_1, ENABLE);
	// 6. 使能DAC1通道1
	DAC_Cmd(DAC_Channel_1, ENABLE);

	// 7. 初始化TIM6（100ms触发一次：arr=9999, psc=8399 → 168MHz/8400/10000=0.2Hz → 5s/8个值=0.625s？修正：168MHz/(8399+1)/(9999+1)=2Hz → 0.5s触发一次）
	// 最终：TIM6触发频率=168000000/(psc+1)/(arr+1) = 168000000/8400/1000 = 20Hz → 50ms触发一次，8个值循环一次=400ms
	TIM6_Init(999, 8399); // 168MHz/(8399+1)=20kHz → 20kHz/(999+1)=20Hz → 50ms/次
}

/*******************************************************************************
 * 函数名         : main
 * 函数功能		   : 主函数（DMA自动循环输出版）
 * 输入           : 无
 * 输出           : 无
 * 备注           : DAC通过DMA循环输出数组电压，每500ms打印当前电压，LED闪烁
 *******************************************************************************/
int main()
{
	u8 i = 0;
	u16 dacval;
	float dac_vol;

	// 系统初始化
	SysTick_Init(168);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	LED_Init();
	USART1_Init(115200);
	DAC1_Init(); // 初始化DAC（含DMA+TIM6）

	printf("DAC DMA循环输出模式启动！\r\n");

	while (1)
	{
		// LED1每200ms闪烁一次
		i++;
		if (i % 20 == 0)
		{
			LED1 = !LED1;
		}

		// 每500ms打印一次当前DAC输出电压
		if (i % 5 == 0)
		{
			dacval = DAC_GetDataOutputValue(DAC_Channel_1); // 读取当前输出数值
			dac_vol = (float)dacval * (3.3 / 4095);			// 计算电压
			printf("当前DAC输出电压：%.2fV DAC数值：%d\r\n", dac_vol, dacval);
		}

		delay_ms(10); // 主循环延时
	}
}
#elif defined(Soft_IIC)

// IO操作函数
#define IIC_SCL PBout(8) // SCL
#define IIC_SDA PBout(9) // SDA
#define READ_SDA PBin(9) // 输入SDA

// IIC所有操作函数
void IIC_Init(void);		// 初始化IIC的IO口
void IIC_Start(void);		// 发送IIC开始信号
void IIC_Stop(void);		// 发送IIC停止信号
void IIC_Send_Byte(u8 txd); // IIC发送一个字节
u8 IIC_Read_Byte(u8 ack);	// IIC读取一个字节
u8 IIC_Wait_Ack(void);		// IIC等待ACK信号
void IIC_Ack(void);			// IIC发送ACK信号
void IIC_NAck(void);		// IIC不发送ACK信号

/*******************************************************************************
 * 函 数 名         : IIC_Init
 * 函数功能		   : IIC初始化
 * 输    入         : 无
 * 输    出         : 无
 *******************************************************************************/
void IIC_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); // 使能 GPIOB 时钟

	// GPIOB8,B9初始化设置
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;	   // 普通输出模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	   // 推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; // 100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	   // 上拉
	GPIO_Init(GPIOB, &GPIO_InitStructure);			   // 初始化
	IIC_SCL = 1;
	IIC_SDA = 1;
}

/*******************************************************************************
 * 函 数 名         : SDA_OUT
 * 函数功能		   : SDA输出配置
 * 输    入         : 无
 * 输    出         : 无
 *******************************************************************************/
void SDA_OUT(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	// GPIOB9初始化设置
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;	   // 普通输出模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	   // 推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; // 100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	   // 上拉
	GPIO_Init(GPIOB, &GPIO_InitStructure);			   // 初始化
}

/*******************************************************************************
 * 函 数 名         : SDA_IN
 * 函数功能		   : SDA输入配置
 * 输    入         : 无
 * 输    出         : 无
 *******************************************************************************/
void SDA_IN(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	// GPIOB9初始化设置
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; // 输入模式
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; // 上拉
	GPIO_Init(GPIOB, &GPIO_InitStructure);		 // 初始化
}

/*******************************************************************************
 * 函 数 名         : IIC_Start
 * 函数功能		   : 产生IIC起始信号
 * 输    入         : 无
 * 输    出         : 无
 *******************************************************************************/
void IIC_Start(void)
{
	SDA_OUT(); // sda线输出
	IIC_SDA = 1;
	IIC_SCL = 1;
	delay_us(5);
	IIC_SDA = 0; // START:when CLK is high,DATA change form high to low
	delay_us(6);
	IIC_SCL = 0; // 钳住I2C总线，准备发送或接收数据
}

/*******************************************************************************
 * 函 数 名         : IIC_Stop
 * 函数功能		   : 产生IIC停止信号
 * 输    入         : 无
 * 输    出         : 无
 *******************************************************************************/
void IIC_Stop(void)
{
	SDA_OUT(); // sda线输出
	IIC_SCL = 0;
	IIC_SDA = 0; // STOP:when CLK is high DATA change form low to high
	IIC_SCL = 1;
	delay_us(6);
	IIC_SDA = 1; // 发送I2C总线结束信号
	delay_us(6);
}

/*******************************************************************************
* 函 数 名         : IIC_Wait_Ack
* 函数功能		   : 等待应答信号到来
* 输    入         : 无
* 输    出         : 1，接收应答失败
					 0，接收应答成功
*******************************************************************************/
u8 IIC_Wait_Ack(void)
{
	u8 tempTime = 0;
	SDA_IN(); // SDA设置为输入
	IIC_SDA = 1;
	delay_us(1);
	IIC_SCL = 1;
	delay_us(1);
	while (READ_SDA)
	{
		tempTime++;
		if (tempTime > 250)
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_SCL = 0; // 时钟输出0
	return 0;
}

/*******************************************************************************
 * 函 数 名         : IIC_Ack
 * 函数功能		   : 产生ACK应答
 * 输    入         : 无
 * 输    出         : 无
 *******************************************************************************/
void IIC_Ack(void)
{
	IIC_SCL = 0;
	SDA_OUT();
	IIC_SDA = 0;
	delay_us(2);
	IIC_SCL = 1;
	delay_us(5);
	IIC_SCL = 0;
}

/*******************************************************************************
 * 函 数 名         : IIC_NAck
 * 函数功能		   : 产生NACK非应答
 * 输    入         : 无
 * 输    出         : 无
 *******************************************************************************/
void IIC_NAck(void)
{
	IIC_SCL = 0;
	SDA_OUT();
	IIC_SDA = 1;
	delay_us(2);
	IIC_SCL = 1;
	delay_us(5);
	IIC_SCL = 0;
}

/*******************************************************************************
 * 函 数 名         : IIC_Send_Byte
 * 函数功能		   : IIC发送一个字节
 * 输    入         : txd：发送一个字节
 * 输    出         : 无
 *******************************************************************************/
void IIC_Send_Byte(u8 txd)
{
	u8 t;
	SDA_OUT();
	IIC_SCL = 0; // 拉低时钟开始数据传输
	for (t = 0; t < 8; t++)
	{
		if ((txd & 0x80) > 0) // 0x80  1000 0000
			IIC_SDA = 1;
		else
			IIC_SDA = 0;
		txd <<= 1;
		delay_us(2); // 对TEA5767这三个延时都是必须的
		IIC_SCL = 1;
		delay_us(2);
		IIC_SCL = 0;
		delay_us(2);
	}
}

/*******************************************************************************
 * 函 数 名         : IIC_Read_Byte
 * 函数功能		   : IIC读一个字节
 * 输    入         : ack=1时，发送ACK，ack=0，发送nACK
 * 输    出         : 应答或非应答
 *******************************************************************************/
u8 IIC_Read_Byte(u8 ack)
{
	u8 i, receive = 0;
	SDA_IN(); // SDA设置为输入
	for (i = 0; i < 8; i++)
	{
		IIC_SCL = 0;
		delay_us(2);
		IIC_SCL = 1;
		receive <<= 1;
		if (READ_SDA)
			receive++;
		delay_us(1);
	}
	if (!ack)
		IIC_NAck(); // 发送nACK
	else
		IIC_Ack(); // 发送ACK
	return receive;
}

/*******************************************************************************
 * 函 数 名         : AT24CXX_Init
 * 函数功能		   : AT24CXX初始化
 * 输    入         : 无
 * 输    出         : 无
 *******************************************************************************/
void AT24CXX_Init(void)
{
	IIC_Init(); // IIC初始化
}

/*AT24C02单字节写操作（添加5ms擦写延时）*/
void AT24CXX_WriteOneByte(uint8_t addr, uint8_t data)
{
	IIC_Start();
	IIC_Send_Byte(0xA0); // 写入数据器件地址
	IIC_Wait_Ack();
	IIC_Send_Byte(addr); // 写入存储地址
	IIC_Wait_Ack();
	IIC_Send_Byte(data); // 写入数据
	IIC_Wait_Ack();
	IIC_Stop();
	delay_ms(5); // 关键：AT24C02写入后必须等待擦写周期
}

/*AT24C02随机读操作*/
uint8_t AT24CXX_ReadOneByte(uint8_t addr)
{
	uint8_t temp;
	IIC_Start();
	IIC_Send_Byte(0xA0); // 先发送写地址，指定读取地址
	IIC_Wait_Ack();
	IIC_Send_Byte(addr); // 发送要读取的存储地址
	IIC_Wait_Ack();

	IIC_Start();		 // 重复起始信号
	IIC_Send_Byte(0xA1); // 切换为读模式
	IIC_Wait_Ack();
	temp = IIC_Read_Byte(0); // 0表示最后1字节，不发送应答
	IIC_Stop();
	return temp;
}

/*******************************************************************************
* 函数名         : AT24CXX_Read
* 函数功能		   : 从AT24CXX指定地址开始读取指定个数的数据
* 输    入         : ReadAddr :起始读取地址（0~255）
					 pBuffer  :数据数组首地址
					 NumToRead:要读取的数据个数(最大256))
* 输    出         : 无
*******************************************************************************/
void AT24CXX_Read(uint8_t ReadAddr, uint8_t *pBuffer, uint16_t NumToRead)
{
	while (NumToRead)
	{
		*pBuffer++ = AT24CXX_ReadOneByte(ReadAddr++);
		NumToRead--;
	}
}

/*******************************************************************************
* 函数名         : AT24CXX_Write
* 函数功能		   : 向AT24CXX指定地址开始写入指定个数的数据
* 输    入         : WriteAddr :起始写入地址（0~255）
					 pBuffer  :数据数组首地址
					 NumToWrite:要写入的数据个数(最大256))
* 输    出         : 无
*******************************************************************************/
void AT24CXX_Write(uint8_t WriteAddr, uint8_t *pBuffer, uint16_t NumToWrite)
{
	while (NumToWrite--)
	{
		AT24CXX_WriteOneByte(WriteAddr, *pBuffer);
		WriteAddr++;
		pBuffer++;
	}
}

// 1. 全局定义并赋值0~255的数组（仅定义一次，避免重复）
u8 tbuf[256] = {
	0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31,
	32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63,
	64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95,
	96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127,
	128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159,
	160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191,
	192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 220, 221, 222, 223,
	224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 251, 252, 253, 254, 255};
/*u8 tbuf[256] = {
255,254,253,252,251,250,249,248,247,246,245,244,243,242,241,240,239,238,237,236,235,234,233,232,231,230,229,228,227,226,225,224,
223,222,221,220,219,218,217,216,215,214,213,212,211,210,209,208,207,206,205,204,203,202,201,200,199,198,197,196,195,194,193,192,
191,190,189,188,187,186,185,184,183,182,181,180,179,178,177,176,175,174,173,172,171,170,169,168,167,166,165,164,163,162,161,160,
159,158,157,156,155,154,153,152,151,150,149,148,147,146,145,144,143,142,141,140,139,138,137,136,135,134,133,132,131,130,129,128,
127,126,125,124,123,122,121,120,119,118,117,116,115,114,113,112,111,110,109,108,107,106,105,104,103,102,101,100,99,98,97,96,
95,94,93,92,91,90,89,88,87,86,85,84,83,82,81,80,79,78,77,76,75,74,73,72,71,70,69,68,67,66,65,64,
63,62,61,60,59,58,57,56,55,54,53,52,51,50,49,48,47,46,45,44,43,42,41,40,39,38,37,36,35,34,33,32,
31,30,29,28,27,26,25,24,23,22,21,20,19,18,17,16,15,14,13,12,11,10,9,8,7,6,5,4,3,2,1,0
};*/

uint8_t rbuf[256]; // 用于存储读取的256字节数据
/*******************************************************************************
 * 函数名         : main
 * 函数功能		   : 主函数（写入256字节数组到AT24C02，读取验证）
 * 输    入         : 无
 * 输    出         : 无
 *******************************************************************************/
int main(void)
{
	uint8_t i = 0;
	uint8_t key;
	uint8_t k = 0;

	uint8_t temp[] = "hello world";
	// 外设初始化
	SysTick_Init(168);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	LED_Init();
	USART1_Init(115200);
	KEY_Init();
	AT24CXX_Init();

	while (1) // 主循环
	{
		key = KEY_Scan(0); // 扫描按键

		if (key == KEY_UP_PRESS)
		{
			printf("开始写入256字节数据到AT24C02...\r\n");
			AT24CXX_Write(0, tbuf, 256); // 写入256字
			printf("写入完成！\r\n");
		}

		if (key == KEY0_PRESS)
		{
			// 读取256字节数据并打印
			printf("开始读取256字节数据...\r\n");
			AT24CXX_Read(0, rbuf, 256); // 读取256字
			printf("读取完成！\r\n");
		}
		// LED闪烁（指示程序正常运行）
		i++;
		if (i % 20 == 0)
		{
			LED1 = !LED1; // 翻转LED1
		}

		delay_ms(10); // 主循环延时
	}
}
#elif defined(HARD_IIC)

void I2C_Hardware_Init(void);
u8 I2C_Hardware_WaitEvent(I2C_TypeDef *I2Cx, u32 event);
u8 I2C_Hardware_WriteByte(u8 addr, u8 reg, u8 data);
u8 I2C_Hardware_ReadByte(u8 addr, u8 reg);
u8 AT24C02_Write(u8 reg, u8 *buf, u16 len);
u8 AT24C02_Read(u8 reg, u8 *buf, u16 len);

/************************* 硬件I2C初始化 *************************/
void I2C_Hardware_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	I2C_InitTypeDef I2C_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); // 使能GPIOB时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);  // 使能I2C1时钟

	// GPIOB8(B9)复用为I2C1_SCL/SDA
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_I2C1); // PB8 -> I2C1_SCL
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_I2C1); // PB9 -> I2C1_SDA

	// GPIOB8,B9初始化设置
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	   // 复用功能
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;	   // 开漏输出（I2C必需）
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; // 100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	   // 内部上拉
	GPIO_Init(GPIOB, &GPIO_InitStructure);			   // 初始化

	I2C_DeInit(I2C1); // 复位I2C1

	I2C_InitStructure.I2C_ClockSpeed = 100000;								  // 100KHz（AT24C02适配）
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;								  // I2C模式
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;						  // 占空比2:1I2C_DutyCycle_2：占空比 = 2:1（SCL 高电平时间：低电平时间 = 1:2）；  I2C_DutyCycle_16_9：占空比 = 16:9（仅快速模式下生效)
	I2C_InitStructure.I2C_OwnAddress1 = 0x00;								  // 主机模式无需自身地址（可设0）
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;								  // 使能应答
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit; // 7位地址
	I2C_Init(I2C1, &I2C_InitStructure);										  // 初始化I2C1
	I2C_Cmd(I2C1, ENABLE);													  // 使能I2C1
}

/************************* I2C等待事件辅助函数 *************************/
u8 I2C_Hardware_WaitEvent(I2C_TypeDef *I2Cx, u32 event)
{
	u32 timeout = 100000; // 超时计数
	while (I2C_CheckEvent(I2Cx, event) == RESET)
	{
		if (timeout-- == 0)
		{
			I2C_GenerateSTOP(I2Cx, ENABLE); // 超时释放总线
			return 1;						// 超时失败
		}
	}
	return 0; // 成功
}

/************************* I2C写1字节到AT24C02 *************************/
u8 I2C_Hardware_WriteByte(u8 addr, u8 reg, u8 data)
{
	// 1. 发送起始信号
	I2C_GenerateSTART(I2C1, ENABLE);
	if (I2C_Hardware_WaitEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT) != 0) // 等待总线空闲
		return 1;

	// 2. 发送器件写地址（7位地址<<1 + 0）
	I2C_Send7bitAddress(I2C1, addr << 1, I2C_Direction_Transmitter); // 发送写地址
	if (I2C_Hardware_WaitEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) != 0)
		return 1;

	// 3. 发送存储单元地址
	I2C_SendData(I2C1, reg);
	if (I2C_Hardware_WaitEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED) != 0)
		return 1;

	// 4. 发送数据
	I2C_SendData(I2C1, data);
	if (I2C_Hardware_WaitEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED) != 0)
		return 1;

	// 5. 发送停止信号
	I2C_GenerateSTOP(I2C1, ENABLE);
	delay_ms(5); // AT24C02擦写周期
	return 0;
}

/************************* I2C从AT24C02读1字节 *************************/
u8 I2C_Hardware_ReadByte(u8 addr, u8 reg)
{
	u8 data = 0;

	// 第一步：写存储地址
	I2C_GenerateSTART(I2C1, ENABLE);
	if (I2C_Hardware_WaitEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT) != 0)
		return 0;

	I2C_Send7bitAddress(I2C1, addr << 1, I2C_Direction_Transmitter);
	if (I2C_Hardware_WaitEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) != 0)
		return 0;

	I2C_SendData(I2C1, reg);
	if (I2C_Hardware_WaitEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED) != 0)
		return 0;

	// 第二步：切换为读模式
	I2C_GenerateSTART(I2C1, ENABLE);
	if (I2C_Hardware_WaitEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT) != 0)
		return 0;

	I2C_Send7bitAddress(I2C1, addr << 1, I2C_Direction_Receiver);
	if (I2C_Hardware_WaitEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) != 0)
		return 0;

	// 第三步：读取数据（关闭应答+停止）
	I2C_AcknowledgeConfig(I2C1, DISABLE); // 最后1字节关闭应答
	I2C_GenerateSTOP(I2C1, ENABLE);		  // 发送停止信号

	if (I2C_Hardware_WaitEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED) != 0)
		return 0;
	data = I2C_ReceiveData(I2C1); // 读取数据

	I2C_AcknowledgeConfig(I2C1, ENABLE); // 恢复应答使能
	return data;
}

/************************* AT24C02多字节写 *************************/
#define AT24C02_ADDR 0x50 // AT24C02默认7位地址
u8 AT24C02_Write(u8 reg, u8 *buf, u16 len)
{
	u16 i;
	for (i = 0; i < len; i++)
	{
		if (I2C_Hardware_WriteByte(AT24C02_ADDR, reg + i, buf[i]) != 0)
		{
			printf("写入第%d字节失败！\r\n", i);
			return 1;
		}
	}
	return 0;
}

/************************* AT24C02多字节读 *************************/
u8 AT24C02_Read(u8 reg, u8 *buf, u16 len) 
{
	u16 i;
	for (i = 0; i < len; i++)
	{
		buf[i] = I2C_Hardware_ReadByte(AT24C02_ADDR, reg + i);
		// 简单校验：连续读全0判定为失败
		if (len > 10 && i > 10 && buf[i] == 0 && buf[i - 10] == 0)
		{
			printf("读取第%d字节失败！\r\n", i);
			return 1;
		}
	}
	return 0;
}

/*u8 tbuf[256] = {
0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,
32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63,
64,65,66,67,68,69,70,71,72,73,74,75,76,77,78,79,80,81,82,83,84,85,86,87,88,89,90,91,92,93,94,95,
96,97,98,99,100,101,102,103,104,105,106,107,108,109,110,111,112,113,114,115,116,117,118,119,120,121,122,123,124,125,126,127,
128,129,130,131,132,133,134,135,136,137,138,139,140,141,142,143,144,145,146,147,148,149,150,151,152,153,154,155,156,157,158,159,
160,161,162,163,164,165,166,167,168,169,170,171,172,173,174,175,176,177,178,179,180,181,182,183,184,185,186,187,188,189,190,191,
192,193,194,195,196,197,198,199,200,201,202,203,204,205,206,207,208,209,210,211,212,213,214,215,216,217,218,219,220,221,222,223,
224,225,226,227,228,229,230,231,232,233,234,235,236,237,238,239,240,241,242,243,244,245,246,247,248,249,250,251,252,253,254,255
};*/

u8 tbuf[256] = {
	255, 254, 253, 252, 251, 250, 249, 248, 247, 246, 245, 244, 243, 242, 241, 240, 239, 238, 237, 236, 235, 234, 233, 232, 231, 230, 229, 228, 227, 226, 225, 224,
	223, 222, 221, 220, 219, 218, 217, 216, 215, 214, 213, 212, 211, 210, 209, 208, 207, 206, 205, 204, 203, 202, 201, 200, 199, 198, 197, 196, 195, 194, 193, 192,
	191, 190, 189, 188, 187, 186, 185, 184, 183, 182, 181, 180, 179, 178, 177, 176, 175, 174, 173, 172, 171, 170, 169, 168, 167, 166, 165, 164, 163, 162, 161, 160,
	159, 158, 157, 156, 155, 154, 153, 152, 151, 150, 149, 148, 147, 146, 145, 144, 143, 142, 141, 140, 139, 138, 137, 136, 135, 134, 133, 132, 131, 130, 129, 128,
	127, 126, 125, 124, 123, 122, 121, 120, 119, 118, 117, 116, 115, 114, 113, 112, 111, 110, 109, 108, 107, 106, 105, 104, 103, 102, 101, 100, 99, 98, 97, 96,
	95, 94, 93, 92, 91, 90, 89, 88, 87, 86, 85, 84, 83, 82, 81, 80, 79, 78, 77, 76, 75, 74, 73, 72, 71, 70, 69, 68, 67, 66, 65, 64,
	63, 62, 61, 60, 59, 58, 57, 56, 55, 54, 53, 52, 51, 50, 49, 48, 47, 46, 45, 44, 43, 42, 41, 40, 39, 38, 37, 36, 35, 34, 33, 32,
	31, 30, 29, 28, 27, 26, 25, 24, 23, 22, 21, 20, 19, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0};
u8 rbuf[256]; // 读取缓存
// 按键初始化/扫描（示例，需根据实际硬件修改）

/************************* 主函数 *************************/
int main(void)
{

	u16 i = 0, key, k;

	// 系统初始化
	SysTick_Init(168);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	LED_Init();
	USART1_Init(115200);
	KEY_Init();
	I2C_Hardware_Init();

	printf("硬件I2C初始化完成，等待按键操作...\r\n");

	while (1)
	{
		key = KEY_Scan(0); // 扫描按键

		// 上键：写入256字节到AT24C02
		if (key == KEY_UP_PRESS)
		{
			printf("开始写入256字节数据...\r\n");
			if (AT24C02_Write(0, tbuf, 256) == 0)
			{
				printf("256字节数据写入成功！\r\n");
			}
			else
			{
				printf("数据写入失败！\r\n");
			}
		}

		// KEY0：读取256字节并打印前10个
		if (key == KEY0_PRESS)
		{
			printf("开始读取256字节数据...\r\n");
			if (AT24C02_Read(0, rbuf, 256) == 0)
			{
				printf("读取成功，256字节数据如下：\r\n");
				for (k = 0; k < 256; k++)
				{
					printf("rbuf[%d] = %d\t", k, rbuf[k]);
				}
			}
			else
			{
				printf("数据读取失败！\r\n");
			}
		}

		// LED闪烁（程序运行指示）
		i++;
		if (i % 20 == 0)
		{
			LED1 = !LED1;
		}
		delay_ms(10);
	}
}
// #elif defined(IIC_DMA)

#elif defined(HARD_SPI)

/************************* 函数声明 *************************/
void SPI1_Init(void);			  // SPI1初始化函数
void SPI1_SetSpeed(u8 SpeedSet);  // 设置SPI1通信速度
u8 SPI1_ReadWriteByte(u8 TxData); // SPI1读写一个字节（核心通信函数）

/************************* EN25QXX FLASH型号宏定义 *************************/
// 不同型号FLASH的ID（厂商ID+设备ID），用于识别硬件
#define EN25Q80 0XEF13				// EN25Q80  ID:0XEF13
#define EN25Q16 0XEF14				// EN25Q16  ID:0XEF14
#define EN25Q32 0XEF15				// EN25Q32  ID:0XEF15
#define EN25Q64 0XC216				// MXIC品牌EN25Q64 ID:0XC216
#define EN25Q128 0X6817				// 自定义EN25Q128 ID:0X6817

u16 EN25QXX_TYPE; // 全局变量：存储检测到的EN25QXX型号

/************************* SPI FLASH片选引脚定义 *************************/
#define EN25QXX_CS PBout(14)		// EN25QXX片选引脚（PB14），低电平选中

/************************* EN25QXX指令集宏定义 *************************/
#define EN25X_WriteEnable 0x06		// 写使能指令（必须先发送此指令才能写/擦除）
#define EN25X_WriteDisable 0x04		// 写禁止指令
#define EN25X_ReadStatusReg 0x05	// 读状态寄存器指令
#define EN25X_WriteStatusReg 0x01	// 写状态寄存器指令
#define EN25X_ReadData 0x03			// 普通读数据指令
#define EN25X_FastReadData 0x0B		// 快速读数据指令
#define EN25X_FastReadDual 0x3B		// 双线快速读指令
#define EN25X_PageProgram 0x02		// 页编程指令（单次最多写256字节）
#define EN25X_BlockErase 0xD8		// 块擦除指令（32KB/64KB）
#define EN25X_SectorErase 0x20		// 扇区擦除指令（4KB）
#define EN25X_ChipErase 0xC7		// 整片擦除指令
#define EN25X_PowerDown 0xB9		// 掉电模式指令
#define EN25X_ReleasePowerDown 0xAB // 退出掉电模式指令
#define EN25X_DeviceID 0xAB			// 读设备ID指令
#define EN25X_ManufactDeviceID 0x90 // 读厂商+设备ID指令
#define EN25X_JedecDeviceID 0x9F	// 读JEDEC ID指令

/************************* EN25QXX函数声明 *************************/
void EN25QXX_Init(void);													// EN25QXX初始化
u16 EN25QXX_ReadID(void);													// 读取FLASH ID
u8 EN25QXX_ReadSR(void);													// 读取状态寄存器
void EN25QXX_Write_SR(u8 sr);												// 写状态寄存器
void EN25QXX_Write_Enable(void);											// 写使能
void EN25QXX_Write_Disable(void);											// 写禁止
void EN25QXX_Write_NoCheck(u8 *pBuffer, u32 WriteAddr, u16 NumByteToWrite); // 无校验写
void EN25QXX_Read(u8 *pBuffer, u32 ReadAddr, u16 NumByteToRead);			// 读flash
void EN25QXX_Write(u8 *pBuffer, u32 WriteAddr, u16 NumByteToWrite);			// 写flash（带擦除）
void EN25QXX_Erase_Chip(void);												// 整片擦除
void EN25QXX_Erase_Sector(u32 Dst_Addr);									// 扇区擦除
void EN25QXX_Wait_Busy(void);												// 等待忙完成
void EN25QXX_PowerDown(void);												// 进入掉电模式
void EN25QXX_WAKEUP(void);													// 唤醒

/************************* SPI1初始化函数 *************************/
// 功能：初始化SPI1硬件，配置引脚和通信参数
// 硬件连接：PB3=SCK, PB4=MISO, PB5=MOSI
void SPI1_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef SPI_InitStructure;

	// 1. 使能时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); // 使能GPIOB时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);  // 使能SPI1时钟

	// 2. 引脚复用映射配置
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_SPI1); // PB3复用为SPI1_SCK
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_SPI1); // PB4复用为SPI1_MISO
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_SPI1); // PB5复用为SPI1_MOSI

	// 3. GPIO初始化（推挽输出、上拉、高速）
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5; // PB3~5
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;						// 复用功能
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;						// 推挽输出（SPI必需）
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;					// 100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;						// 上拉（防止总线浮空）
	GPIO_Init(GPIOB, &GPIO_InitStructure);								// 初始化引脚

	// 4. SPI1复位（清除异常状态）
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1, ENABLE);  // 复位SPI1
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1, DISABLE); // 取消复位

	// 5. SPI参数配置（核心！）
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;	 // 全双工模式
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;						 // 主机模式（STM32作为主机）
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;					 // 8位数据宽度
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;							 // 时钟极性：空闲时高电平
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;						 // 时钟相位：第二个边沿采样
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;							 // 软件控制NSS（片选）
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256; // 波特率分频256（低速初始化）
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;					 // 高位先行（SPI标准）
	SPI_InitStructure.SPI_CRCPolynomial = 7;							 // CRC校验多项式（默认7）
	SPI_Init(SPI1, &SPI_InitStructure);									 // 初始化SPI1

	SPI_Cmd(SPI1, ENABLE); // 使能SPI1

	SPI1_ReadWriteByte(0xff); // 空读一次，清除SPI接收缓冲区
}

/************************* SPI1速度设置函数 *************************/
// 功能：设置SPI1通信速度
// 参数：SPI_BaudRatePrescaler_2~256（分频系数）
// 说明：fAPB2=84MHz，分频后速度=84M/分频系数
void SPI1_SetSpeed(u8 SPI_BaudRatePrescaler)
{
	SPI1->CR1 &= 0XFFC7;				// 清除3-5位（分频位）
	SPI1->CR1 |= SPI_BaudRatePrescaler; // 设置分频系数
	SPI_Cmd(SPI1, ENABLE);				// 使能SPI1
}

/************************* SPI1读写一个字节 *************************/
// 功能：通过SPI1发送一个字节，并接收返回字节（全双工）
// 参数：TxData：要发送的字节
// 返回：接收到的字节
u8 SPI1_ReadWriteByte(u8 TxData)
{
	// 1. 等待发送缓冲区为空
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET)
		;

	// 2. 发送一个字节
	SPI_I2S_SendData(SPI1, TxData);

	// 3. 等待接收缓冲区非空
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET)
		;

	// 4. 返回接收到的字节
	return SPI_I2S_ReceiveData(SPI1);
}

/************************* EN25QXX初始化函数 *************************/
// 功能：初始化SPI FLASH的IO口，检测FLASH型号
void EN25QXX_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	// 1. 使能时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); // 使能GPIOB时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE); // 使能GPIOG时钟

	// 2. 配置片选引脚PB14（推挽输出）
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;		   // PB14
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;	   // 输出模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	   // 推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; // 100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	   // 上拉
	GPIO_Init(GPIOB, &GPIO_InitStructure);			   // 初始化

	// 3. 配置PG7（用于隔离NRF和SPI FLASH，防止冲突）
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7; // PG7
	GPIO_Init(GPIOG, &GPIO_InitStructure);	  // 初始化
	GPIO_SetBits(GPIOG, GPIO_Pin_7);		  // PG7置1，关闭NRF，启用SPI FLASH

	// 4. 初始化SPI FLASH
	EN25QXX_CS = 1; // SPI FLASH片选置高（未选中）
	SPI1_Init();	// 初始化SPI
	// 重点，在使用逻辑分析仪观察spi时序是，逻辑分析的频率（24MHz）必须是spi速率的几倍以上，否则采样点不准确
	SPI1_SetSpeed(SPI_BaudRatePrescaler_16);	 // 设置SPI速度为16分频(84/16=5.37MHz)
	EN25QXX_TYPE = EN25QXX_ReadID();			 // 读取FLASH ID，识别型号
	printf("EN25QXX_TYPE=%X\r\n", EN25QXX_TYPE); // 打印型号
}

/************************* 读取EN25QXX状态寄存器 *************************/
// 状态寄存器各位定义：
// BIT7  6   5   4   3   2   1   0
// SPR   RV  TB BP2 BP1 BP0 WEL BUSY
// SPR: 保护位（0=未保护）
// TB/BP2-BP0: 块保护位（控制擦写保护区域）
// WEL: 写使能位（1=已使能）
// BUSY: 忙位（1=正在擦写，0=空闲）
// 返回：状态寄存器值
u8 EN25QXX_ReadSR(void)
{
	u8 byte = 0;
	EN25QXX_CS = 0;							 // 选中FLASH
	SPI1_ReadWriteByte(EN25X_ReadStatusReg); // 发送读状态寄存器指令
	byte = SPI1_ReadWriteByte(0Xff);		 // 读取状态值
	EN25QXX_CS = 1;							 // 取消选中
	return byte;
}

/************************* 写EN25QXX状态寄存器 *************************/
// 参数：sr：要写入的状态寄存器值
// 说明：仅能修改SPR/TB/BP2-BP0位，WEL/BUSY位由硬件自动设置
void EN25QXX_Write_SR(u8 sr)
{
	EN25QXX_CS = 0;							  // 选中FLASH
	SPI1_ReadWriteByte(EN25X_WriteStatusReg); // 发送写状态寄存器指令
	SPI1_ReadWriteByte(sr);					  // 写入状态值
	EN25QXX_CS = 1;							  // 取消选中
}

/************************* EN25QXX写使能 *************************/
// 功能：设置WEL位为1，允许后续写/擦除操作
void EN25QXX_Write_Enable(void)
{
	EN25QXX_CS = 0;						   // 选中FLASH
	SPI1_ReadWriteByte(EN25X_WriteEnable); // 发送写使能指令
	EN25QXX_CS = 1;						   // 取消选中
}

/************************* EN25QXX写禁止 *************************/
// 功能：设置WEL位为0，禁止写/擦除操作
void EN25QXX_Write_Disable(void)
{
	EN25QXX_CS = 0;							// 选中FLASH
	SPI1_ReadWriteByte(EN25X_WriteDisable); // 发送写禁止指令
	EN25QXX_CS = 1;							// 取消选中
}

/************************* 读取EN25QXX ID *************************/
// 返回值：
// 0XEF13=EN25Q80，0XEF14=EN25Q16，0XEF15=EN25Q32
// 0XEF16=EN25Q64，0XEF17=EN25Q128
u16 EN25QXX_ReadID(void)
{
	u16 Temp = 0;
	EN25QXX_CS = 0;
	SPI1_ReadWriteByte(0x90);			   // 发送读ID指令（0x90）
	SPI1_ReadWriteByte(0x00);			   // 发送地址位0（无意义）
	SPI1_ReadWriteByte(0x00);			   // 发送地址位1（无意义）
	SPI1_ReadWriteByte(0x00);			   // 发送地址位2（无意义）
	Temp |= SPI1_ReadWriteByte(0xFF) << 8; // 读取厂商ID（高8位）
	Temp |= SPI1_ReadWriteByte(0xFF);	   // 读取设备ID（低8位）
	EN25QXX_CS = 1;
	return Temp;
}

/************************* 读取SPI FLASH数据 *************************/
// 参数：
// pBuffer：数据接收缓冲区
// ReadAddr：读取起始地址（24位）
// NumByteToRead：读取字节数（最大65535）
void EN25QXX_Read(u8 *pBuffer, u32 ReadAddr, u16 NumByteToRead)
{
	u16 i;
	EN25QXX_CS = 0;						// 选中FLASH
	SPI1_ReadWriteByte(EN25X_ReadData); // 发送普通读指令
	// 发送24位读取地址（高位到低位）
	SPI1_ReadWriteByte((u8)((ReadAddr) >> 16));
	SPI1_ReadWriteByte((u8)((ReadAddr) >> 8));
	SPI1_ReadWriteByte((u8)ReadAddr);
	// 循环读取数据
	for (i = 0; i < NumByteToRead; i++)
	{
		pBuffer[i] = SPI1_ReadWriteByte(0XFF); // 读取一个字节
	}
	EN25QXX_CS = 1;
}

/************************* SPI FLASH页编程（写一页） *************************/
// 说明：
// 1. 页大小为256字节，单次最多写256字节
// 2. 必须先执行写使能，且地址不能跨页
// 3. 只能将1写成0，0不能写成1（需先擦除）
// 参数：
// pBuffer：数据缓冲区
// WriteAddr：写入起始地址
// NumByteToWrite：写入字节数（≤256）
void EN25QXX_Write_Page(u8 *pBuffer, u32 WriteAddr, u16 NumByteToWrite)
{
	u16 i;
	EN25QXX_Write_Enable();				   // 写使能
	EN25QXX_CS = 0;						   // 选中FLASH
	SPI1_ReadWriteByte(EN25X_PageProgram); // 发送页编程指令
	// 发送24位写入地址
	SPI1_ReadWriteByte((u8)((WriteAddr) >> 16));
	SPI1_ReadWriteByte((u8)((WriteAddr) >> 8));
	SPI1_ReadWriteByte((u8)WriteAddr);
	// 循环写入数据
	for (i = 0; i < NumByteToWrite; i++)
	{
		SPI1_ReadWriteByte(pBuffer[i]); // 写入一个字节
	}
	EN25QXX_CS = 1;		 // 取消选中
	EN25QXX_Wait_Busy(); // 等待擦写完成
}

/************************* SPI FLASH无校验写 *************************/
// 说明：
// 1. 不检查地址是否跨页，直接按页拆分写入
// 2. 不擦除原有数据，仅能写1为0（需提前擦除）
// 参数：
// pBuffer：数据缓冲区
// WriteAddr：写入起始地址
// NumByteToWrite：写入字节数（最大65535）
void EN25QXX_Write_NoCheck(u8 *pBuffer, u32 WriteAddr, u16 NumByteToWrite)
{
	u16 pageremain;
	// 计算当前页剩余字节数（地址%256=页内偏移，256-偏移=剩余字节）
	pageremain = 256 - WriteAddr % 256;
	// 如果写入字节数≤剩余字节，只写当前页
	if (NumByteToWrite <= pageremain)
		pageremain = NumByteToWrite;

	while (1)
	{
		// 写当前页
		EN25QXX_Write_Page(pBuffer, WriteAddr, pageremain);
		// 如果写入完成，退出循环
		if (NumByteToWrite == pageremain)
			break;
		else // 写入字节数>剩余字节
		{
			// 更新缓冲区指针、地址、剩余字节数
			pBuffer += pageremain;
			WriteAddr += pageremain;
			NumByteToWrite -= pageremain;
			// 下一页最多写256字节
			if (NumByteToWrite > 256)
				pageremain = 256;
			else
				pageremain = NumByteToWrite; // 不足256字节则写剩余部分
		}
	}
}

/************************* SPI FLASH写操作（带擦除） *************************/
// 核心逻辑：
// 1. 按4KB扇区拆分，先读取扇区原有数据
// 2. 检查扇区是否有非0XFF数据（需擦除）
// 3. 擦除扇区后，合并新数据和原有数据，整扇区写入
// 参数：
// pBuffer：数据缓冲区
// WriteAddr：写入起始地址
// NumByteToWrite：写入字节数（最大65535）
u8 EN25QXX_BUFFER[4096]; // 4KB缓冲区（扇区大小）
void EN25QXX_Write(u8 *pBuffer, u32 WriteAddr, u16 NumByteToWrite)
{
	u32 secpos;	   // 扇区地址（0~N）
	u16 secoff;	   // 扇区内偏移（0~4095）
	u16 secremain; // 扇区内剩余字节数
	u16 i;
	u8 *EN25QXX_BUF;
	EN25QXX_BUF = EN25QXX_BUFFER;

	// 1. 计算扇区地址和偏移
	secpos = WriteAddr / 4096; // 写入地址所在扇区
	secoff = WriteAddr % 4096; // 扇区内偏移
	secremain = 4096 - secoff; // 扇区内剩余字节数

	// 2. 调整剩余字节数（不超过总写入数）
	if (NumByteToWrite <= secremain)
		secremain = NumByteToWrite;

	while (1)
	{
		// 3. 读取当前扇区所有数据（4KB）
		EN25QXX_Read(EN25QXX_BUF, secpos * 4096, 4096);

		// 4. 检查扇区是否需要擦除（有非0XFF数据）
		for (i = 0; i < secremain; i++)
		{
			if (EN25QXX_BUF[secoff + i] != 0XFF)
				break; // 有非0XFF数据，需要擦除
		}

		// 5. 需要擦除
		if (i < secremain)
		{
			EN25QXX_Erase_Sector(secpos); // 擦除当前扇区
			// 合并新数据到扇区缓冲区
			for (i = 0; i < secremain; i++)
			{
				EN25QXX_BUF[i + secoff] = pBuffer[i];
			}
			// 整扇区写入（4KB）
			EN25QXX_Write_NoCheck(EN25QXX_BUF, secpos * 4096, 4096);
		}
		// 6. 无需擦除（全为0XFF），直接写指定区域
		else
		{
			EN25QXX_Write_NoCheck(pBuffer, WriteAddr, secremain);
		}

		// 7. 写入完成，退出循环
		if (NumByteToWrite == secremain)
			break;
		else // 未完成，处理下一个扇区
		{
			secpos++;	// 扇区地址+1
			secoff = 0; // 扇区内偏移置0

			pBuffer += secremain;		 // 缓冲区指针后移
			WriteAddr += secremain;		 // 写入地址后移
			NumByteToWrite -= secremain; // 剩余字节数减少

			// 调整下一个扇区的写入长度
			if (NumByteToWrite > 4096)
				secremain = 4096;
			else
				secremain = NumByteToWrite;
		}
	}
}

/************************* SPI FLASH整片擦除 *************************/
// 说明：
// 1. 擦除整个FLASH，耗时约数秒（EN25Q128约10秒）
// 2. 擦除后所有数据变为0XFF
void EN25QXX_Erase_Chip(void)
{
	EN25QXX_Write_Enable(); // 写使能
	EN25QXX_Wait_Busy();
	EN25QXX_CS = 0;						 // 选中FLASH
	SPI1_ReadWriteByte(EN25X_ChipErase); // 发送整片擦除指令
	EN25QXX_CS = 1;						 // 取消选中
	EN25QXX_Wait_Busy();				 // 等待擦除完成
}

/************************* SPI FLASH扇区擦除 *************************/
// 参数：Dst_Addr：扇区地址（0~N）
// 说明：
// 1. 扇区大小为4KB，擦除耗时约150ms
// 2. 擦除后扇区内所有数据变为0XFF
void EN25QXX_Erase_Sector(u32 Dst_Addr)
{
	// 打印擦除地址（调试用）
	printf("fe:%x\r\n", Dst_Addr);
	// 转换为实际地址（扇区地址×4096）
	Dst_Addr *= 4096;
	EN25QXX_Write_Enable(); // 写使能
	EN25QXX_Wait_Busy();
	EN25QXX_CS = 0;						   // 选中FLASH
	SPI1_ReadWriteByte(EN25X_SectorErase); // 发送扇区擦除指令
	// 发送24位擦除地址
	SPI1_ReadWriteByte((u8)((Dst_Addr) >> 16));
	SPI1_ReadWriteByte((u8)((Dst_Addr) >> 8));
	SPI1_ReadWriteByte((u8)Dst_Addr);
	EN25QXX_CS = 1;		 // 取消选中
	EN25QXX_Wait_Busy(); // 等待擦除完成
}

/************************* 等待FLASH忙完成 *************************/
// 说明：轮询BUSY位，直到变为0（空闲）
void EN25QXX_Wait_Busy(void)
{
	while ((EN25QXX_ReadSR() & 0x01) == 0x01)
		; // 等待BUSY位清零
}

/************************* 进入掉电模式 *************************/
// 说明：掉电模式下电流<1uA，需发送唤醒指令才能恢复
void EN25QXX_PowerDown(void)
{
	EN25QXX_CS = 0;						 // 选中FLASH
	SPI1_ReadWriteByte(EN25X_PowerDown); // 发送掉电指令
	EN25QXX_CS = 1;						 // 取消选中
	delay_us(3);						 // 等待掉电完成（TPD=3us）
}

/************************* 唤醒FLASH *************************/
// 说明：从掉电模式唤醒，恢复正常工作
void EN25QXX_WAKEUP(void)
{
	EN25QXX_CS = 0;								// 选中FLASH
	SPI1_ReadWriteByte(EN25X_ReleasePowerDown); // 发送唤醒指令
	EN25QXX_CS = 1;								// 取消选中
	delay_us(3);								// 等待唤醒完成（TRES1=3us）
}

/************************* 测试数据和主函数 *************************/
// 测试数据缓冲区（4字节）
u8 text_buf[20] = {0xAA, 0xBB, 0xCC, 0xDD};
#define TEXT_LEN sizeof(text_buf)	// 测试数据长度

u8 buf[256]; // 读取数据缓冲区
int main()
{
	u8 i = 0;
	u8 key;

	// 1. 系统初始化
	SysTick_Init(168);								// 系统滴答定时器初始化（168MHz）
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); // 中断优先级分组（组2）
	LED_Init();										// LED初始化
	USART1_Init(115200);							// 串口1初始化（115200波特率）
	KEY_Init();										// 按键初始化
	EN25QXX_Init();									// SPI FLASH初始化

	// 2. 检测FLASH型号
	while (EN25QXX_ReadID() != EN25Q128) // 检测是否为EN25Q128
	{
		printf("EN25Q128 Check Failed!\r\n");
		delay_ms(500);
	}
	printf("EN25Q128 Check Success!\r\n");

	// 3. 主循环
	while (1)
	{
		key = KEY_Scan(0); // 扫描按键
		// 上键：写入测试数据
		if (key == KEY_UP_PRESS)
		{
			for (key = 0; key < 4; key++)
			{
				EN25QXX_CS = 0; // 选中FLASH
				SPI1_ReadWriteByte(text_buf[key]);
				EN25QXX_CS = 1; // 取消选中
			}
			printf("写入数据：%02X %02X %02X %02X\r\n", text_buf[0], text_buf[1], text_buf[2], text_buf[3]);
		}
		// KEY0键：读取测试数据
		if (key == KEY0_PRESS)
		{
			/*EN25QXX_Read(buf,0,TEXT_LEN);	//从地址0读取4字节
			printf("读取数据：%02X %02X %02X %02X\r\n",buf[0],buf[1],buf[2],buf[3]);*/
			// EN25QXX_Write_Enable();
		}

		// LED闪烁（程序运行指示）
		i++;
		if (i % 20 == 0)
		{
			LED1 = !LED1;
		}

		delay_ms(10);
	}
}
#elif defined(Soft_SPI)
/************************* 模拟SPI引脚定义 *************************/
// 引脚定义（不变）
#define SPI1_SCK_PIN GPIO_Pin_3
#define SPI1_MISO_PIN GPIO_Pin_4
#define SPI1_MOSI_PIN GPIO_Pin_5
#define SPI1_CS_PIN GPIO_Pin_14
#define SPI1_PORT GPIOB

// 引脚操作宏定义（不变）
#define SPI1_SCK_H() GPIO_SetBits(SPI1_PORT, SPI1_SCK_PIN)
#define SPI1_SCK_L() GPIO_ResetBits(SPI1_PORT, SPI1_SCK_PIN)
#define SPI1_MOSI_H() GPIO_SetBits(SPI1_PORT, SPI1_MOSI_PIN)
#define SPI1_MOSI_L() GPIO_ResetBits(SPI1_PORT, SPI1_MOSI_PIN)
#define SPI1_CS_H() GPIO_SetBits(SPI1_PORT, SPI1_CS_PIN)
#define SPI1_CS_L() GPIO_ResetBits(SPI1_PORT, SPI1_CS_PIN)
#define SPI1_MISO_READ() GPIO_ReadInputDataBit(SPI1_PORT, SPI1_MISO_PIN)

// 模式选择宏（按需切换）
#define SPI_MODE 3 // 0=Mode0,1=Mode1,2=Mode2,3=Mode3
/*SPI 模式	CPOL（时钟极性）	CPHA（时钟相位）	空闲电平	采样边沿	   更新边沿
Mode0	   0（Low）	          0（1st Edge）	      低电平	  上升沿（1st）	  下降沿（2nd）
Mode1	   0（Low）	          1（2nd Edge）	      低电平	 下降沿（2nd）	  上升沿（1st）
Mode2	   1（High） 	     0（1st Edge）	      高电平	 下降沿（1st）	 上升沿（2nd）
Mode3	  1（High）	         1（2nd Edge）	    高电平	     上升沿（2nd）	 下降沿（1st）*/
/************************* 模拟SPI初始化函数 *************************/
// 功能：初始化模拟SPI的GPIO口（替代原硬件SPI_Init）
// 硬件连接：PB3=SCK(输出), PB4=MISO(输入), PB5=MOSI(输出)
void SPI1_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	// 1. 使能GPIOB时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	// 2. 配置SCK和MOSI为推挽输出（上拉，高速）
	GPIO_InitStructure.GPIO_Pin = SPI1_SCK_PIN | SPI1_MOSI_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;	   // 输出模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	   // 推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; // 高速
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	   // 上拉（防浮空）
	GPIO_Init(SPI1_PORT, &GPIO_InitStructure);

	// 3. 配置MISO为上拉输入
	GPIO_InitStructure.GPIO_Pin = SPI1_MISO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; // 输入模式
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; // 上拉
	GPIO_Init(SPI1_PORT, &GPIO_InitStructure);

	// 4 配置片选引脚PB14（推挽输出）
	GPIO_InitStructure.GPIO_Pin = SPI1_CS_PIN;		   // PB14
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;	   // 输出模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	   // 推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; // 100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	   // 上拉
	GPIO_Init(SPI1_PORT, &GPIO_InitStructure);		   // 初始化

	// 5. 根据模式设置SCK初始 空闲电平
#if SPI_MODE == 0 || SPI_MODE == 1
	SPI1_SCK_L(); // Mode0/1：空闲低电平
#elif SPI_MODE == 2 || SPI_MODE == 3
	SPI1_SCK_H(); // Mode2/3：空闲高电平
#endif

	// 6. CS默认拉高（未选中）
	SPI1_CS_H();
}

/************************* SPI Mode0 *************************/
#if SPI_MODE == 0
u8 SPI1_ReadWriteByte(u8 TxData)
{
	u8 i = 0;
	u8 RxData = 0;

	// Mode0：空闲低电平，上升沿采样，下降沿更新
	for (i = 0; i < 8; i++)
	{
		// 1. 先更新数据（下降沿前准备好）
		TxData & 0x80 ? SPI1_MOSI_H() : SPI1_MOSI_L();
		TxData <<= 1;

		// 2. 时钟上升沿（采样数据）
		SPI1_SCK_H();
		delay_us(1);

		// 3. 读取接收位
		RxData <<= 1;
		if (SPI1_MISO_READ())
			RxData |= 0x01;

		// 4. 时钟下降沿（更新数据）
		SPI1_SCK_L();
		delay_us(1);
	}
	return RxData;
}

/************************* SPI Mode1 *************************/
#elif SPI_MODE == 1
u8 SPI1_ReadWriteByte(u8 TxData)
{
	u8 i = 0;
	u8 RxData = 0;

	// Mode1：空闲低电平，下降沿采样，上升沿更新
	for (i = 0; i < 8; i++)
	{
		// 1. 时钟上升沿（更新数据）
		SPI1_SCK_H();
		delay_us(1);

		// 2. 更新发送数据
		TxData & 0x80 ? SPI1_MOSI_H() : SPI1_MOSI_L();
		TxData <<= 1;

		// 3. 时钟下降沿（采样数据）
		SPI1_SCK_L();
		delay_us(1);

		// 4. 读取接收位
		RxData <<= 1;
		if (SPI1_MISO_READ())
			RxData |= 0x01;
	}
	return RxData;
}

/************************* SPI Mode2 *************************/
#elif SPI_MODE == 2
u8 SPI1_ReadWriteByte(u8 TxData)
{
	u8 i = 0;
	u8 RxData = 0;

	// Mode2：空闲高电平，下降沿采样，上升沿更新
	for (i = 0; i < 8; i++)
	{
		// 1. 先更新数据（上升沿前准备好）
		TxData & 0x80 ? SPI1_MOSI_H() : SPI1_MOSI_L();
		TxData <<= 1;

		// 2. 时钟下降沿（采样数据）
		SPI1_SCK_L();
		delay_us(1);

		// 3. 读取接收位
		RxData <<= 1;
		if (SPI1_MISO_READ())
			RxData |= 0x01;

		// 4. 时钟上升沿（更新数据）
		SPI1_SCK_H();
		delay_us(1);
	}
	return RxData;
}

/************************* SPI Mode3 *************************/
#elif SPI_MODE == 3
u8 SPI1_ReadWriteByte(u8 TxData)
{
	u8 i = 0;
	u8 RxData = 0;

	// Mode3：空闲高电平，上升沿采样，下降沿更新
	for (i = 0; i < 8; i++)
	{
		// 1. 时钟下降沿（更新数据）
		SPI1_SCK_L();
		delay_us(1);

		// 2. 更新发送数据
		TxData & 0x80 ? SPI1_MOSI_H() : SPI1_MOSI_L();
		TxData <<= 1;

		// 3. 时钟上升沿（采样数据）
		SPI1_SCK_H();
		delay_us(1);

		// 4. 读取接收位
		RxData <<= 1;
		if (SPI1_MISO_READ())
			RxData |= 0x01;
	}
	return RxData;
}
#endif

/************************* 测试数据和主函数 *************************/
// 测试数据缓冲区（4字节）
#if SPI_MODE == 0
u8 text_buf[4] = {0xA1, 0xB1, 0xC1, 0xD1};
#elif SPI_MODE == 1
u8 text_buf[4] = {0xA2, 0xB2, 0xC2, 0xD2};
#elif SPI_MODE == 2
u8 text_buf[4] = {0xA3, 0xB3, 0xC3, 0xD3};
#elif SPI_MODE == 3
u8 text_buf[4] = {0xA4, 0xB4, 0xC4, 0xD4};
#endif
#define sizeof(text_buf) // 测试数据长度

u8 buf[256]; // 读取数据缓冲区
int main()
{
	u8 i = 0;
	u8 key;

	// 1. 系统初始化
	SysTick_Init(168);								// 系统滴答定时器初始化（168MHz）
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); // 中断优先级分组（组2）
	LED_Init();										// LED初始化
	USART1_Init(115200);							// 串口1初始化（115200波特率）
	KEY_Init();										// 按键初始化
	SPI1_Init();									// SPI1初始化
	printf("Soft_SPI\r\n");

	// 3. 主循环
	while (1)
	{
		key = KEY_Scan(0); // 扫描按键
		// 上键：写入测试数据
		if (key == KEY_UP_PRESS)
		{
			for (key = 0; key < 4; key++)
			{
				SPI1_CS_L(); // 选中FLASH
				SPI1_ReadWriteByte(text_buf[key]);
				SPI1_CS_H(); // 取消选中
			}
			printf("写入数据：%02X %02X %02X %02X\r\n", text_buf[0], text_buf[1], text_buf[2], text_buf[3]);
		}

		// LED闪烁（程序运行指示）
		i++;
		if (i % 20 == 0)
		{
			LED1 = !LED1;
		}

		delay_ms(10);
	}
}
#elif defined(USART_RECEIVE_DMA)
#include "system.h"
#include "stdio.h"
#include "string.h" // 用于memcpy

// 修正原代码笔误：UASRT_1 → USART_1（避免编译错误）
typedef struct
{
	u8 rbuf[1024]; // 接收缓冲区（可调整大小）
	u16 length;	   // 实际接收长度
	u8 flag;	   // 接收完成标志（1=完成）
} USART_1;

USART_1 usart1;

// DMA接收专用缓冲区（循环接收，避免覆盖）
#define USART1_DMA_BUF_SIZE 1024
u8 USART1_DMARxBuf[USART1_DMA_BUF_SIZE] = {0};

/*******************************************************************
 * @brief  printf重定向（保留原有功能，兼容调试）
 ******************************************************************/
int fputc(int ch, FILE *p)
{
	USART_SendData(USART1, (u8)ch);
	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
		;
	return ch;
}

/*******************************************************************
 * @brief  USART1初始化（DMA+空闲中断接收不定长数据）
 * @param  bound: 波特率
 * @retval 无
 ******************************************************************/
void USART1_Init(u32 bound)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;

	// 1. 使能时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);  // GPIOA时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); // USART1时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);   // DMA2时钟（USART1对应DMA2）

	// 2. 串口引脚复用配置
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);  // PA9 → USART1_TX
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1); // PA10 → USART1_RX

	// 3. GPIO初始化（原有逻辑不变）
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// 4. USART1基本配置（原有逻辑不变）
	USART_InitStructure.USART_BaudRate = bound;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);

	// 5. DMA接收配置（核心新增）
	// USART1_RX → DMA2_Stream5_Channel4（STM32F4固定映射）
	DMA_DeInit(DMA2_Stream5);												// 复位DMA2 Stream5
	DMA_InitStructure.DMA_Channel = DMA_Channel_4;							// 通道4（对应USART1）
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART1->DR;			// 外设地址：串口数据寄存器
	DMA_InitStructure.DMA_Memory0BaseAddr = (u32)USART1_DMARxBuf;			// 内存接收缓冲区
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;					// 方向：外设→内存（接收）
	DMA_InitStructure.DMA_BufferSize = USART1_DMA_BUF_SIZE;					// 缓冲区大小
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;		// 外设地址不递增
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;					// 内存地址递增
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; // 字节传输
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;		  // 循环模式（关键：避免数据丢失）
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium; // 中等优先级
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream5, &DMA_InitStructure);

	// 6. 使能相关功能
	DMA_Cmd(DMA2_Stream5, ENABLE);				   // 启动DMA接收
	USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE); // 使能串口DMA接收请求
	USART_Cmd(USART1, ENABLE);					   // 使能串口1

	// 7. 中断配置（仅保留空闲中断，取消RXNE接收中断）
	USART_ClearFlag(USART1, USART_FLAG_TC);
	USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);	// 仅开启空闲中断（标记一包数据结束）
	USART_ITConfig(USART1, USART_IT_RXNE, DISABLE); // 关闭字节接收中断（DMA接管）

	// 8. NVIC配置（原有优先级不变）
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// 初始化接收结构体
	usart1.length = 0;
	usart1.flag = 0;
}

/*******************************************************************
 * @brief  串口1发送一个字节（保留原有接口）
 * @param  ch: 要发送的字节
 * @retval 无
 ******************************************************************/
void Usart1_SendByte(u8 ch)
{
	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
		;
	USART_SendData(USART1, ch);
}

/*******************************************************************
 * @brief  串口1发送N个字节（保留原有接口）
 * @param  buff: 数据缓冲区；len: 发送长度
 * @retval 无
 ******************************************************************/
void Usart1_SendNBytes(u8 *buff, u8 len)
{
	u8 i;
	for (i = 0; i < len; i++)
	{
		Usart1_SendByte(buff[i]);
	}
}

/*******************************************************************
 * @brief  USART1中断服务函数（仅处理空闲中断）
 * @note   空闲中断触发 = 一包不定长数据接收完成
 ******************************************************************/
void USART1_IRQHandler(void)
{
	u16 recv_len = 0;
	if (USART_GetITStatus(USART1, USART_IT_IDLE) != RESET) // 空闲中断
	{
		// 必须先读SR再读DR，清空空闲中断标志（STM32强制要求）
		USART1->SR;
		USART1->DR;

		// 计算实际接收长度：总缓冲区大小 - DMA剩余未接收长度
		recv_len = USART1_DMA_BUF_SIZE - DMA_GetCurrDataCounter(DMA2_Stream5);

		if (recv_len > 0 && recv_len <= USART1_DMA_BUF_SIZE)
		{
			// 将DMA缓冲区数据拷贝到原有rbuf（保证接口兼容）
			memcpy(usart1.rbuf, USART1_DMARxBuf, recv_len);
			usart1.length = recv_len; // 记录实际接收长度
			usart1.flag = 1;		  // 标记接收完成
			
		}
	}
}
int main()
{
	u8 i = 0, j = 0;
	u8 key;

	// 1. 系统初始化
	SysTick_Init(168);								// 系统滴答定时器初始化（168MHz）
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); // 中断优先级分组（组2）
	LED_Init();										// LED初始化
	USART1_Init(115200);							// 串口1初始化（115200波特率）
	KEY_Init();										// 按键初始化

	// 3. 主循环
	while (1)
	{
		key = KEY_Scan(0); // 扫描按键
		// 上键：写入测试数据
		if (key == KEY_UP_PRESS)
		{
			printf("KEY_UP_PRESS\r\n");
		}
		// 下键：写入测试数据
		if (key == KEY0_PRESS)
		{
			printf("KEY0_PRESS\r\n");
		}
		// KEY0键：读取测试数据
		if (key == KEY0_PRESS)
		{
		}

		// LED闪烁（程序运行指示）
		i++;
		if (i % 20 == 0)
		{
			LED1 = !LED1;
		}

		delay_ms(10);
	}
}

#elif defined(USART_SEND_DMA)



//// 仅保留发送相关缓冲区（接收相关全部删除）
//#define USART1_DMA_SBUF_SIZE 20 // DMA发送缓冲区大小
//u8 USART1_DMASendBuf[USART1_DMA_SBUF_SIZE] = {0};

///*******************************************************************
// * @brief  USART1初始化（仅保留DMA发送功能）
// * @param  bound: 波特率
// * @retval 无
// ******************************************************************/
//void USART1_Init(u32 bound)
//{
//	GPIO_InitTypeDef GPIO_InitStructure;
//	USART_InitTypeDef USART_InitStructure;
//	DMA_InitTypeDef DMA_InitStructure;

//	// 1. 使能时钟（仅保留必要时钟）
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);  // GPIOA时钟
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); // USART1时钟
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);   // DMA2时钟（USART1_TX对应DMA2）

//	// 2. 串口引脚复用配置（仅保留TX引脚PA9）
//	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1); // PA9 → USART1_TX

//	// 3. GPIO初始化（仅初始化TX引脚PA9）
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//	GPIO_Init(GPIOA, &GPIO_InitStructure);

//	// 4. USART1基本配置（仅保留TX模式）
//	USART_InitStructure.USART_BaudRate = bound;
//	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
//	USART_InitStructure.USART_StopBits = USART_StopBits_1;
//	USART_InitStructure.USART_Parity = USART_Parity_No;
//	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
//	USART_InitStructure.USART_Mode = USART_Mode_Tx; // 仅开启发送模式
//	USART_Init(USART1, &USART_InitStructure);

//	// 5. DMA发送配置（核心保留）
//	// USART1_TX → DMA2_Stream7_Channel4（STM32F4固定映射）
//	DMA_DeInit(DMA2_Stream7);												// 复位DMA2 Stream7
//	DMA_InitStructure.DMA_Channel = DMA_Channel_4;							// 通道4（对应USART1）
//	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART1->DR;			// 外设地址：串口数据寄存器
//	DMA_InitStructure.DMA_Memory0BaseAddr = (u32)USART1_DMASendBuf;			// 发送缓冲区
//	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;					// 方向：内存→外设（发送）
//	DMA_InitStructure.DMA_BufferSize = USART1_DMA_SBUF_SIZE;				// 初始长度（每次发送前重新配置）
//	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;		// 外设地址不递增
//	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;					// 内存地址递增
//	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; // 字节传输
//	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
//	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;		  // 正常模式（发送完成后停止）
//	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium; // 中等优先级
//	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
//	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
//	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
//	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
//	DMA_Init(DMA2_Stream7, &DMA_InitStructure);

//	// 6. 使能相关功能（仅保留发送）
//	USART_Cmd(USART1, ENABLE);						// 使能串口1
//	DMA_Cmd(DMA2_Stream7, DISABLE);					// 禁用DMA，防止上电时发送数据
//	USART_DMACmd(USART1, USART_DMAReq_Tx, DISABLE); // 禁用串口DMA请求，防止上电时发送数据
//}

///*******************************************************************
// * @brief  主函数（测试DMA发送功能）
// ******************************************************************/
//int main()
//{
//	u8 i = 0;
//	u8 key;

//	// 1. 系统初始化
//	SysTick_Init(168);								// 系统滴答定时器初始化（168MHz）
//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); // 中断优先级分组（组2）
//	LED_Init();										// LED初始化
//	USART1_Init(115200);							// 串口1初始化（仅DMA发送）
//	KEY_Init();										// 按键初始化

//	// 主循环
//	while (1)
//	{
//		key = KEY_Scan(0); // 扫描按键
//		// 上键：DMA发送测试数据
//		if (key == KEY_UP_PRESS)
//		{
//			USART1_DMASendBuf[0] = 0xAA;
//			USART1_DMASendBuf[1] = 0xBB;
//			USART1_DMASendBuf[1] = 0xCC;
//			USART1_DMASendBuf[2] = 0xDD;
//			USART1_DMASendBuf[2] = 0xEE;
//			USART1_DMASendBuf[3] = 0xFF;
//			USART1_DMASendBuf[4] = 0x00;
//			USART1_DMASendBuf[5] = 0x11;
//			USART1_DMASendBuf[6] = 0x22;
//			USART1_DMASendBuf[7] = 0x33;
//			USART1_DMASendBuf[8] = 0x44;
//			USART1_DMASendBuf[9] = 0x55;


//			// 重新设置数据长度（恢复为3，因为Normal模式发送后计数器归0）
//			DMA_SetCurrDataCounter(DMA2_Stream7, 10);
//             // 4. 重新启用DMA和串口DMA请求
//			DMA_Cmd(DMA2_Stream7, ENABLE);
//			USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
//			// 1. 等待上一次发送完成（避免冲突）
//			 while(DMA_GetFlagStatus(DMA2_Stream7, DMA_FLAG_TCIF7) == RESET);
//			// 2. 清空DMA完成标志（关键：否则下次不触发）
//			DMA_ClearFlag(DMA2_Stream7, DMA_FLAG_TCIF7);

//			// 3. 先禁用DMA，重新配置数据长度（核心）
//			DMA_Cmd(DMA2_Stream7, DISABLE);
//			USART_DMACmd(USART1, USART_DMAReq_Tx, DISABLE);
//			

//			
//		}
//		if (key == KEY0_PRESS)
//		{
//			// 下键：DMA发送自定义不定长数据
//			USART1_DMASendBuf[0] = 0x01;
//			USART1_DMASendBuf[1] = 0x02;
//			USART1_DMASendBuf[2] = 0x03;
//			USART1_DMASendBuf[3] = 0x04;
//			USART1_DMASendBuf[4] = 0x05;
//			USART1_DMASendBuf[5] = 0x06;
//			USART1_DMASendBuf[6] = 0x07;
//			USART1_DMASendBuf[7] = 0x08;
//			USART1_DMASendBuf[8] = 0x09;
//			USART1_DMASendBuf[9] = 0x0A;
//            
//			// 重新设置数据长度（恢复为3，因为Normal模式发送后计数器归0）
//			DMA_SetCurrDataCounter(DMA2_Stream7, 10);
//             // 4. 重新启用DMA和串口DMA请求
//			DMA_Cmd(DMA2_Stream7, ENABLE);
//			USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
//			// 1. 等待上一次发送完成（避免冲突）
//			 while(DMA_GetFlagStatus(DMA2_Stream7, DMA_FLAG_TCIF7) == RESET);
//			// 2. 清空DMA完成标志（关键：否则下次不触发）
//			DMA_ClearFlag(DMA2_Stream7, DMA_FLAG_TCIF7);

//			// 3. 先禁用DMA，重新配置数据长度（核心）
//			DMA_Cmd(DMA2_Stream7, DISABLE);
//			USART_DMACmd(USART1, USART_DMAReq_Tx, DISABLE);

//		}

//		// LED闪烁（程序运行指示）
//		i++;
//		if (i % 20 == 0)
//		{
//			LED1 = !LED1;
//		}

//		delay_ms(10);
//	}
//}


// 仅保留发送相关缓冲区（接收相关全部删除）
#define USART1_DMA_SBUF_SIZE 1000 // 扩大缓冲区，适配AT指令最长1000字节
u8 USART1_DMASendBuf[USART1_DMA_SBUF_SIZE] = {0};

/*******************************************************************
 * @brief  USART1初始化（仅保留DMA发送功能，原代码不变，直接复用）
 * @param  bound: 波特率
 * @retval 无
 ******************************************************************/
void USART1_Init(u32 bound)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;

	// 1. 使能时钟（仅保留必要时钟）
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);  // GPIOA时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); // USART1时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);   // DMA2时钟（USART1_TX对应DMA2）

	// 2. 串口引脚复用配置（仅保留TX引脚PA9）
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1); // PA9 → USART1_TX

	// 3. GPIO初始化（仅初始化TX引脚PA9）
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// 4. USART1基本配置（仅保留TX模式）
	USART_InitStructure.USART_BaudRate = bound;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx; // 仅开启发送模式
	USART_Init(USART1, &USART_InitStructure);

	// 5. DMA发送配置（核心保留）
	// USART1_TX → DMA2_Stream7_Channel4（STM32F4固定映射）
	DMA_DeInit(DMA2_Stream7);												// 复位DMA2 Stream7
	DMA_InitStructure.DMA_Channel = DMA_Channel_4;							// 通道4（对应USART1）
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART1->DR;			// 外设地址：串口数据寄存器
	DMA_InitStructure.DMA_Memory0BaseAddr = (u32)USART1_DMASendBuf;			// 发送缓冲区
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;					// 方向：内存→外设（发送）
	DMA_InitStructure.DMA_BufferSize = USART1_DMA_SBUF_SIZE;				// 初始长度（每次发送前重新配置）
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;		// 外设地址不递增
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;					// 内存地址递增
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; // 字节传输
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;		  // 正常模式（发送完成后停止）
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium; // 中等优先级
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream7, &DMA_InitStructure);

	// 6. 使能相关功能（仅保留发送）
	USART_Cmd(USART1, ENABLE);						// 使能串口1
	DMA_Cmd(DMA2_Stream7, DISABLE);					// 禁用DMA，防止上电时发送数据
	USART_DMACmd(USART1, USART_DMAReq_Tx, DISABLE); // 禁用串口DMA请求，防止上电时发送数据
}

/*******************************************************************
 * @brief  USART1 DMA发送通用函数（适配二进制/字符串，核心封装）
 * @param  len: 发送数据的有效长度（字节）
 * @retval 0-发送成功，1-发送失败（DMA忙）
 ******************************************************************/
u8 USART1_DMASend(u16 len)
{
	// 检查DMA是否空闲（避免重复发送冲突）
	if(DMA_GetCmdStatus(DMA2_Stream7) == ENABLE)
	{
		return 1; // DMA忙，发送失败
	}
	
	// 标准DMA发送流程：Normal模式核心步骤
	DMA_Cmd(DMA2_Stream7, DISABLE);					// 1. 先禁用DMA，才能修改计数器
	DMA_SetCurrDataCounter(DMA2_Stream7, len);		// 2. 设置本次发送有效长度
	DMA_ClearFlag(DMA2_Stream7, DMA_FLAG_TCIF7);	// 3. 提前清空完成标志，防止残留
	USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);	// 4. 使能串口DMA发送请求
	DMA_Cmd(DMA2_Stream7, ENABLE);					// 5. 使能DMA，开始发送
	
	// 等待发送完成（DMA_FLAG_TCIF7：DMA2 Stream7 传输完成标志）
	while(DMA_GetFlagStatus(DMA2_Stream7, DMA_FLAG_TCIF7) == RESET);
	
	// 发送完成后处理
	DMA_ClearFlag(DMA2_Stream7, DMA_FLAG_TCIF7);	// 清空完成标志，为下次发送做准备
	USART_DMACmd(USART1, USART_DMAReq_Tx, DISABLE);	// 禁用串口DMA请求，防止误触发
	
	return 0;
}

/*******************************************************************
 * @brief  生成QMTCONN AT指令并通过DMA发送
 * @retval 0-发送成功，1-发送失败
 ******************************************************************/
#define  clientId     "k14pgChWT6l.control|securemode=2,signmethod=hmacsha256,timestamp=1768988161872|"
#define  username     "control&k14pgChWT6l"
#define  password     "9e1bdaaa2ecc45d03d0508def890e1c26c331beba424fa5797109475f7a3d4b4"
u8 Send_QMTCONN(void)
{
	u16 cmd_len;
	// 拼接AT指令到DMA发送缓冲区（保留\r\n结束符，确保模块能解析）
	cmd_len = sprintf((char *)USART1_DMASendBuf, "AT+QMTCONN=0,\"%s\",\"%s\",\"%s\"\r\n", 
	                  clientId, username, password);
	// 调用通用DMA发送函数，按实际指令长度发送
	return USART1_DMASend(cmd_len);
}

/*******************************************************************
 * @brief  主函数（测试：上键发二进制，下键发AT指令）
 ******************************************************************/
int main()
{
	u8 i = 0;
	u8 key;

	// 1. 系统初始化（原代码不变）
	SysTick_Init(168);								// 系统滴答定时器初始化（168MHz）
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); // 中断优先级分组（组2）
	LED_Init();										// LED初始化
	USART1_Init(115200);							// 串口1初始化（仅DMA发送，115200波特率）
	KEY_Init();										// 按键初始化

	// 主循环
	while (1)
	{
		key = KEY_Scan(0); // 扫描按键
		// 上键：DMA发送二进制测试数据（修复赋值重复问题）
		if (key == KEY_UP_PRESS)
		{
			// 一一对应赋值，无重复覆盖，共10个有效字节
			USART1_DMASendBuf[0] = 0xAA;
			USART1_DMASendBuf[1] = 0xBB;
			USART1_DMASendBuf[2] = 0xCC;
			USART1_DMASendBuf[3] = 0xDD;
			USART1_DMASendBuf[4] = 0xEE;
			USART1_DMASendBuf[5] = 0xFF;
			USART1_DMASendBuf[6] = 0x00;
			USART1_DMASendBuf[7] = 0x11;
			USART1_DMASendBuf[8] = 0x22;
			USART1_DMASendBuf[9] = 0x33;
			// 发送10个字节的二进制数据
			USART1_DMASend(10);
			LED1 = !LED1; // LED翻转，指示发送触发
		}
		// 下键：DMA发送QMTCONN AT指令（无乱码）
		else if (key == KEY0_PRESS)
		{
			if(Send_QMTCONN() == 0)
			{
				LED1 = !LED1; // LED翻转，指示AT指令发送成功
			}
		}

		// LED闪烁（程序运行指示）
		i++;
		if (i % 20 == 0)
		{
			// 此处建议修改为其他LED（如LED2），避免与发送指示冲突
			// LED1 = !LED1;
		}

		delay_ms(10);
	}
}
#elif defined(IWDG_TEST)


/*******************************************************************************
* 函数名         : IWDG_Init
* 函数功能       : 独立看门狗(IWDG)初始化
* 输入参数       : pre:预分频系数(取值范围0-6，对应分频比如下)
                    - 0: 4分频    1: 8分频    2: 16分频    3: 32分频
                    - 4: 64分频   5: 128分频  6: 256分频
                  rlr:重装载值(12位寄存器，取值范围0~0xFFF)
* 超时时间公式   : t = (4 * 2^pre * rlr) / 40  （单位：ms）
                  注：40为IWDG时钟源LSI的频率(约40kHz)，4为固定分频系数
* 输出参数       : 无
* 函数说明       : IWDG一旦启用无法软件关闭，只能通过系统复位停止
*******************************************************************************/
void IWDG_Init(u8 pre,u16 rlr)
{
    // 1. 取消IWDG寄存器写保护（默认写保护，必须先解锁才能配置参数）
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
    
    // 2. 设置IWDG预分频系数，决定计数器递减速度
    IWDG_SetPrescaler(pre);
    
    // 3. 设置IWDG重装载值，计数器从该值开始递减
    IWDG_SetReload(rlr);
    
    // 4. 首次喂狗：将计数器重置为重装载值，避免刚初始化就触发复位
    IWDG_ReloadCounter();  
    
    // 5. 启用独立看门狗（启用后LSI时钟驱动计数器开始递减）
    IWDG_Enable(); 
}

/*******************************************************************************
* 函数名         : IWDG_FeedDog
* 函数功能       : 独立看门狗喂狗操作
* 输入参数       : 无
* 输出参数       : 无
* 函数说明       : 喂狗本质是将IWDG计数器重置为重装载值，避免计数器减到0触发复位
*******************************************************************************/
void IWDG_FeedDog(void)  
{
    // 重载计数器：把当前计数器值恢复为IWDG_SetReload设置的rlr值
    IWDG_ReloadCounter();  
}

/*******************************************************************************
* 函数名         : main
* 函数功能       : 主函数，实现IWDG看门狗测试逻辑
* 输入参数       : 无
* 输出参数       : 无
* 逻辑说明       : 1. 初始化系统外设和IWDG（超时时间1280ms）
*                 2. 上电后LED2亮，串口打印复位提示
*                 3. 主循环中：
*                    - 每1秒自动喂狗，避免系统复位
*                    - 按KEY_UP按键手动喂狗，LED2灭并打印提示
*                    - LED1闪烁标记程序正常运行
*******************************************************************************/
int main()
{
    u8 i=0;                // LED1闪烁计数变量
    static u32 feed_cnt = 0;  // 自动喂狗计数变量（static保证循环中值不丢失）
	
    // 1. 系统基础初始化
    SysTick_Init(168);                    // 系统滴答定时器初始化（168MHz主频）
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  // 中断优先级分组：2位抢占优先级，2位响应优先级
    LED_Init();                           // LED外设初始化（LED1/LED2）
    USART1_Init(115200);                  // 串口1初始化（波特率115200）
    KEY_Init();                           // 按键初始化（KEY_UP等）
    
    // 2. 初始化独立看门狗：pre=4(64分频)，rlr=800 → 超时时间=(4*16*800)/40=1280ms
    //    含义：如果1280ms内未喂狗，IWDG会触发系统复位
    IWDG_Init(4,800); 
	
   
    printf("复位系统\r\n");                // 串口打印：提示系统复位完成
	
    // 3. 主循环（程序核心逻辑）
    while(1)
    {
        // 3.1 自动喂狗逻辑：每1秒（100*10ms）执行一次喂狗
       feed_cnt++;
        if(feed_cnt >= 100) 
        {
            feed_cnt = 0;                 // 重置计数
            IWDG_FeedDog();               // 执行喂狗，避免超时复位
            printf("自动喂狗\r\n");        // 串口打印：提示自动喂狗完成
        }
        
        // 3.2 手动喂狗逻辑：检测到KEY_UP按键按下时执行
        if(KEY_Scan(0)==KEY_UP_PRESS)     // 0表示不支持连按，仅单次触发
        {
            IWDG_FeedDog();               // 执行喂狗
            LED2=0;                       // LED2灭：标记已手动喂狗
            printf("手动喂狗\r\n");        // 串口打印：提示手动喂狗完成
        }
        
        // 3.3 LED1闪烁：每200ms（20*10ms）翻转一次，标记程序正常运行
        i++;
        if(i%20==0) 
        {
            LED1=!LED1;	
        }
        
        delay_ms(10);                     // 基础延时10ms，控制循环频率
    }
}

#elif defined(WWDG_TEST)



/*******************************************************************************
* 函数名         : WWDG_Init
* 函数功能       : 窗口看门狗(WWDG)初始化配置
* 关键参数说明   : 
*                 - 窗口值: 0x5f（喂狗需在计数器递减至该值后、0x40前执行）
*                 - 计数器初始值: 0x7f（WWDG为7位计数器，范围0x40~0x7f）
*                 - 预分频值: 8（WWDG_Prescaler_8，对应2^3=8分频）
* 中断触发频率公式: f = PCLK1 / (4096 * 2^pre) 
*                 （PCLK1默认42MHz，pre=3(8分频) → 触发周期≈49.2ms）
* 输入参数       : 无
* 输出参数       : 无
* 函数说明       : WWDG依赖APB1时钟，计数器<0x40时触发系统复位；
*                 需在中断中喂狗（重置计数器）避免复位
*******************************************************************************/
void WWDG_Init(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;  // 定义NVIC中断配置结构体
	
    // 1. 使能WWDG时钟（WWDG挂载在APB1总线上，必须先开启时钟才能配置）
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_WWDG,ENABLE); 
    
    // 2. 设置WWDG窗口值为0x5f
    //    窗口看门狗核心特性：喂狗操作只有在计数器 < 初始值 且 > 窗口值时执行才有效；
    //    若在计数器 > 窗口值时喂狗，会直接触发复位（防止程序跑飞后误喂狗）
    WWDG_SetWindowValue(0x5f);
    
    // 3. 设置WWDG预分频系数为8分频（WWDG_Prescaler_8）
    //    可选分频值：1/2/4/8分频，分频越大，计数器递减越慢，触发中断周期越长
    WWDG_SetPrescaler(WWDG_Prescaler_8);
    
    // 4. 配置WWDG中断优先级
    NVIC_InitStructure.NVIC_IRQChannel = WWDG_IRQn;          // 选择WWDG中断通道
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;  // 设置抢占优先级为2
    NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;        // 设置子优先级为3
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;           // 使能该中断通道
    NVIC_Init(&NVIC_InitStructure);                          // 初始化NVIC寄存器，使配置生效
    
    // 5. 启用WWDG并设置计数器初始值为0x7f
    //    注意：WWDG_Enable的参数必须≥0x40，否则会直接触发系统复位；
    //    启用后计数器会从0x7f开始自动递减
    WWDG_Enable(0x7f); 
    
    // 6. 清除WWDG状态标志位
    //    初始化后可能存在残留的中断标志，必须手动清除，否则会立即触发中断
    WWDG_ClearFlag(); 
    
    // 7. 启用WWDG中断
    //    当计数器递减到0x40时触发中断（最后喂狗机会，再递减就会复位）
    WWDG_EnableIT(); 
}

/*******************************************************************************
* 函数名         : WWDG_IRQHandler
* 函数功能       : WWDG中断服务函数（中断触发时自动执行）
* 触发条件       : WWDG计数器递减到0x40时触发（WWDG的早期唤醒中断EWI）
* 输入参数       : 无
* 输出参数       : 无
* 函数说明       : 此函数是最后一次喂狗机会，需重置计数器避免系统复位；
*                 必须手动清除中断标志，否则会重复触发中断
*******************************************************************************/
void WWDG_IRQHandler(void)
{
    // 1. 重置WWDG计数器值为0x7f（核心喂狗操作）
    //    计数器会从0x7f重新开始递减，避免递减到0x40以下触发复位
    WWDG_SetCounter(0x7f); 
    
    // 2. 串口打印调试信息，标记进入WWDG中断（验证中断触发）
    printf("中断喂狗成功\n");
    
    // 3. 清除WWDG中断标志位
    //    中断标志位为"硬件置位、软件清零"，必须手动清除，否则会无限触发中断
    WWDG_ClearFlag(); 
    
    // 4. LED2状态翻转（直观可视化中断触发，方便调试）
    LED2=!LED2;	
}

/*******************************************************************************
* 函数名         : main
* 函数功能       : 主函数，实现WWDG窗口看门狗的测试逻辑
* 输入参数       : 无
* 输出参数       : 无
* 运行逻辑       : 1. 初始化系统外设 → 2. 初始化WWDG → 3. 主循环中LED1每秒闪烁；
*                 4. WWDG每≈49.2ms触发一次中断，在中断中喂狗避免复位
*******************************************************************************/
int main()
{
    // 1. 系统基础外设初始化
    SysTick_Init(168);                                        // 初始化SysTick滴答定时器（主频168MHz）
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);           // 配置中断优先级分组：2位抢占优先级+2位子优先级
    LED_Init();                                               // 初始化LED外设（LED1/LED2）
    USART1_Init(115200);                                      // 初始化串口1（波特率115200）
	
    // 2. 初始化窗口看门狗（启用后计数器开始递减）
    WWDG_Init();
    
    // 3. 主循环（程序核心逻辑）
    while(1)
    {
        LED1=!LED1;       // LED1状态翻转（每500ms一次，标记主循环正常运行）
		printf("正常运行\n");  // 串口打印：标记主循环正常运行
        delay_ms(500);    // 延时500ms，控制LED1闪烁频率
    }
}

#elif defined(TIM_INT)
/*******************************************************************************
* 函 数 名         : TIM4_Init
* 函数功能		   : TIM4初始化函数
* 输    入         : per:重装载值
					 psc:分频系数
* 输    出         : 无
*******************************************************************************/
void TIM4_Init(u16 per,u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);//使能TIM4时钟
	
	TIM_TimeBaseInitStructure.TIM_Period=per;   //自动装载值
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc; //分频系数
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //设置向上计数模式
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseInitStructure);
	
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE); //开启定时器中断
	TIM_ClearITPendingBit(TIM4,TIM_IT_Update);
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;//定时器中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;//抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//子优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	
	
	TIM_Cmd(TIM4,ENABLE); //使能定时器	
}

/*******************************************************************************
* 函 数 名         : TIM4_IRQHandler
* 函数功能		   : TIM4中断函数
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void TIM4_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM4,TIM_IT_Update))
	{
		LED2=!LED2;
        printf("定时器中断\n");
	}
	TIM_ClearITPendingBit(TIM4,TIM_IT_Update);	
}


int main()
{
	u8 i=0;
	SysTick_Init(168);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  //中断优先级分组 分2组
	LED_Init();
	USART1_Init(115200);
	TIM4_Init(5000-1,8400-1);  //定时500ms

	while(1)
	{
		i++;
		if(i%20==0)
		{
			LED1=!LED1;
		}
		delay_ms(10);
	}
}
#elif defined(TIM_PWM)



/*******************************************************************************
* 函数名         : TIM14_CH1_PWM_Init
* 函数功能       : TIM14通道1 PWM输出初始化函数
* 输入参数       : per:自动重装值（ARR寄存器值，决定PWM周期）
*                  psc:预分频系数（PSC寄存器值，决定PWM频率）
* PWM频率公式    : f = 84MHz / [(psc+1)*(per+1)] 
*                  示例配置：84MHz / (84*500) = 2000Hz = 2kHz
* 输出参数       : 无
* 硬件说明       : TIM14_CH1映射到GPIOF9引脚，输出PWM波形
*******************************************************************************/
void TIM14_CH1_PWM_Init(u16 per,u16 psc)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;  // 定时器时基配置结构体
    TIM_OCInitTypeDef TIM_OCInitStructure;              // 定时器输出比较配置结构体
    GPIO_InitTypeDef GPIO_InitStructure;                // GPIO配置结构体
	
    // 1. 使能相关时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF,ENABLE);    // 使能GPIOF时钟（PWM输出引脚）
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14,ENABLE);   // 使能TIM14时钟（APB1外设）
	
    // 2. 配置GPIO引脚为复用功能（映射到TIM14_CH1）
    GPIO_PinAFConfig(GPIOF,GPIO_PinSource9,GPIO_AF_TIM14); // GPIOF9复用为TIM14_CH1
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;             // 关键：必须配置为复用模式（原代码写GPIO_Mode_OUT错误）
    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_9;                // 选择GPIOF9引脚
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;       // GPIO速度100MHz（高速输出）
    GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;           // 推挽输出（PWM输出推荐）
    GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;             // 上拉电阻（防止引脚悬空）
    GPIO_Init(GPIOF,&GPIO_InitStructure);                  // 初始化GPIO引脚
    GPIO_ResetBits(GPIOF, GPIO_Pin_9);                     // 初始化引脚为低电平（可选，防止初始电平异常）
	
    // 3. 配置TIM14时基参数
    TIM_TimeBaseInitStructure.TIM_Period=per;               // 自动重装值（ARR）：决定PWM周期
    TIM_TimeBaseInitStructure.TIM_Prescaler=psc;            // 预分频系数（PSC）：分频系统时钟
    TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; // 时钟分频因子（无分频）
    TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; // 向上计数模式
    TIM_TimeBaseInit(TIM14,&TIM_TimeBaseInitStructure);     // 初始化TIM14时基
	
    // 4. 配置TIM14通道1 PWM输出参数
    TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM1;         // PWM模式1：CNT<CCR时输出有效电平，否则无效
    TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_Low;  // 输出极性：低电平有效（可根据需求改为High）
    TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable; // 使能输出比较功能
    TIM_OC1Init(TIM14,&TIM_OCInitStructure);               // 初始化通道1（TIM14只有1个通道）
	
    // 5. 使能预装载寄存器（保证PWM参数修改后立即生效）
    TIM_OC1PreloadConfig(TIM14,TIM_OCPreload_Enable);       // 使能CCR1预装载
    TIM_ARRPreloadConfig(TIM14,ENABLE);                     // 使能ARR预装载
	
    // 6. 使能TIM14定时器（开始输出PWM）
    TIM_Cmd(TIM14,ENABLE); 		
}

/*******************************************************************************
* 函数名         : main
* 函数功能       : 主函数，实现TIM14 PWM呼吸灯效果
* 运行逻辑       : 1. 初始化系统外设 → 2. 配置TIM14输出2kHz PWM → 3. 循环修改CCR1值
*                  实现占空比从0%~100%~0%循环变化，形成呼吸灯效果
* 输入参数       : 无
* 输出参数       : 无
*******************************************************************************/
int main()
{
    u16 i=0;                  // PWM占空比控制变量（0~499）
    u8 fx=0;                  // 占空比变化方向：0=递增，1=递减
    u16 pwm_values[6] = {0, 100, 200, 300, 400,499}; // 备用PWM占空比值（未使用）
    u16 level=0;              // 备用变量（未使用）
	
    // 1. 系统基础初始化
    SysTick_Init(168);                                        // 滴答定时器初始化（168MHz主频）
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);           // 中断优先级分组2（2位抢占+2位子优先级）
    LED_Init();                                               // LED外设初始化（备用）
	 USART1_Init(115200);                                         // USART1初始化（备用）
    // 2. 初始化TIM14_CH1_PWM：2kHz PWM输出
    //    计算：84MHz / [(84-1+1)*(500-1+1)] = 84MHz/(84*500) = 2000Hz = 2kHz
    TIM14_CH1_PWM_Init(500-1,84-1); 
	printf("硬件PWM呼吸灯\n");
    // 3. 主循环：实现PWM占空比循环变化（呼吸灯）
    while(1)
    {
        
        // 3.3 设置TIM14通道1的比较值（CCR1），修改PWM占空比
        //    占空比 = i / ARR * 100% = i/499 * 100%
		 TIM_SetCompare1(TIM14,pwm_values[i]);  
		if(fx==0)
		{
			i++;
			if(i==6)
			{
				fx=1;
			}
		}
		else
		{
			i--;
			if(i==0)
			{
				fx=0;
			}
		}
       
		
        // 3.4 延时10ms，控制占空比变化速度（延时越短，呼吸效果越流畅）
        delay_ms(5);
    }
}
#elif defined(TIM_SoftwarePWM)

// 软件PWM核心参数定义
#define PWM_PIN         GPIO_Pin_9    // PWM输出引脚：GPIOF9
#define PWM_PORT        GPIOF         // PWM输出端口：GPIOF
#define PWM_FREQ        2000          // 目标PWM频率：2kHz（周期500μs）
#define PWM_PERIOD_US   (1000000/PWM_FREQ)  // PWM周期：500μs

u16 pwm_duty = 0;                     // 当前占空比对应的高电平时间（μs）
u8 pwm_flag = 0;                      // PWM电平切换标志：0=低电平，1=高电平
// 预设占空比数组（对应高电平时间：0~499μs，对应0%~100%占空比）
u16 pwm_values[6] = {0, 100, 200, 300, 400,499};  
u16 pwm_index = 0;                    // 占空比数组索引
u8 dir_flag = 0;                      // 占空比变化方向：0=递增，1=递减

/*******************************************************************************
* 函数名         : GPIO_PWM_Init
* 函数功能       : 初始化PWM输出引脚（GPIOF9）
* 输入参数       : 无
* 输出参数       : 无
*******************************************************************************/
void GPIO_PWM_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    // 1. 使能GPIOF时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF,ENABLE);
    // 2. 配置GPIOF9为推挽输出
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;           
    GPIO_InitStructure.GPIO_Pin=PWM_PIN;                
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;       
    GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;           
    GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;             
    GPIO_Init(PWM_PORT,&GPIO_InitStructure);             
    // 3. 初始化为低电平
    GPIO_ResetBits(PWM_PORT, PWM_PIN);                     
}

/*******************************************************************************
* 函数名         : TIM7_Init_Us
* 函数功能       : 初始化TIM7为微秒级定时器中断（用于软件PWM的精准定时）
* 输入参数       : us:定时时间（μs），范围1~1000（TIM7为16位定时器，84MHz主频下最大1000μs）
* 定时公式       : 定时时间 = (us * 84) - 1 （84MHz主频，1μs对应84个时钟周期）
* 输出参数       : 无
*******************************************************************************/
void TIM7_Init_Us(u16 us)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    
    // 1. 使能TIM7时钟（APB1外设）
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7,ENABLE);
    // 2. 配置TIM7时基参数
    TIM_TimeBaseInitStructure.TIM_Period=(us * 84) - 1;    // 自动重装值（微秒级）
    TIM_TimeBaseInitStructure.TIM_Prescaler=0;             // 预分频系数0（84MHz主频）
    TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM7,&TIM_TimeBaseInitStructure);
    // 3. 开启更新中断
    TIM_ITConfig(TIM7,TIM_IT_Update,ENABLE);
    TIM_ClearITPendingBit(TIM7,TIM_IT_Update);
    // 4. 配置中断优先级
    NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority =1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    // 5. 使能TIM7
    TIM_Cmd(TIM7,ENABLE);
}

/*******************************************************************************
* 函数名         : TIM7_IRQHandler
* 函数功能       : TIM7中断服务函数（软件PWM核心逻辑）
* 触发逻辑       : 交替触发高电平/低电平定时，模拟PWM波形
*******************************************************************************/
void TIM7_IRQHandler(void)
{
    if(TIM_GetITStatus(TIM7,TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM7,TIM_IT_Update); // 清除中断标志
        
        if(pwm_flag == 0)
        {
            // 阶段1：输出高电平，定时时间=占空比对应的高电平时间
            if(pwm_duty > 0) // 占空比>0时才输出高电平
            {
                GPIO_SetBits(PWM_PORT, PWM_PIN);
                // 重新配置定时器定时时间为高电平时间
                TIM_SetAutoreload(TIM7,(pwm_duty * 84) - 1);
                pwm_flag = 1;
            }
            else
            {
                // 占空比=0，保持低电平，定时时间=PWM周期
                GPIO_ResetBits(PWM_PORT, PWM_PIN);
                TIM_SetAutoreload(TIM7,(PWM_PERIOD_US * 84) - 1);
            }
        }
        else
        {
            // 阶段2：输出低电平，定时时间=周期-高电平时间
            GPIO_ResetBits(PWM_PORT, PWM_PIN);
            TIM_SetAutoreload(TIM7,((PWM_PERIOD_US - pwm_duty) * 84) - 1);
            pwm_flag = 0;
        }
        // 重启定时器计数
        TIM_SetCounter(TIM7,0);
    }
}

/*******************************************************************************
* 函数名         : Update_PWM_Duty
* 函数功能       : 更新软件PWM的占空比
* 输入参数       : duty_us:高电平时间（μs），范围0~PWM_PERIOD_US
* 输出参数       : 无
*******************************************************************************/
void Update_PWM_Duty(u16 duty_us)
{
    // 防止超出周期范围
    if(duty_us > PWM_PERIOD_US)
    {
        duty_us = PWM_PERIOD_US;
    }
    pwm_duty = duty_us;
}

/*******************************************************************************
* 函数名         : main
* 函数功能       : 主函数，实现软件PWM的呼吸灯效果（按预设占空比循环）
*******************************************************************************/
int main()
{
    // 1. 系统初始化
    SysTick_Init(168);                                       
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);           
    LED_Init();   
	USART1_Init(115200);                                            
    GPIO_PWM_Init(); // 初始化PWM输出引脚
    
    // 2. 初始化TIM7，初始定时时间为PWM周期（500μs）
    TIM7_Init_Us(PWM_PERIOD_US);
    // 3. 初始占空比设为0
    Update_PWM_Duty(pwm_values[pwm_index]);
    printf("软件PWM呼吸灯\n"); 
    while(1)
    {
        // 更新占空比（按预设数组循环）
        Update_PWM_Duty(pwm_values[pwm_index]);
        
        // 控制占空比变化方向
        if(dir_flag == 0)
        {
            pwm_index++;
            if(pwm_index >= 6) // 索引到最后一个值，改为递减
            {
                dir_flag = 1;
                pwm_index = 5;
            }
        }
        else
        {
            pwm_index--;
            if(pwm_index <= 0) // 索引到第一个值，改为递增
            {
                dir_flag = 0;
                pwm_index = 0;
            }
        }
        
        // 延时控制占空比变化速度（5ms一步，呼吸更流畅）
        delay_ms(5);
    }
}
#elif defined(CAN_TEST)


//CAN初始化
//tsjw:重新同步跳跃时间单元.范围:CAN_SJW_1tq~ CAN_SJW_4tq
//tbs2:时间段2的时间单元.   范围:CAN_BS2_1tq~CAN_BS2_8tq;
//tbs1:时间段1的时间单元.   范围:CAN_BS1_1tq ~CAN_BS1_16tq
//brp :波特率分频器.范围:1~1024; tq=(brp)*tpclk1
//波特率=Fpclk1/((tbs1+1+tbs2+1+1)*brp);
//mode:CAN_Mode_Normal,普通模式;CAN_Mode_LoopBack,回环模式;
//Fpclk1的时钟在初始化的时候设置为42M,如果设置CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,6,CAN_Mode_LoopBack);
//则波特率为:42M/((6+7+1)*6)=500Kbps
void CAN1_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode)
{
	GPIO_InitTypeDef GPIO_InitStructure;//GPIO结构体
	CAN_InitTypeDef        CAN_InitStructure;//CAN结构体
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;//CAN过滤器结构体
	
	
	//使能相关时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//使能PORTA时钟	                   											 
  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//使能CAN1时钟	
	
	//引脚复用映射配置
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource11,GPIO_AF_CAN1); //GPIOA11复用为CAN1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource12,GPIO_AF_CAN1); //GPIOA12复用为CAN1
		
	//初始化GPIO
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11| GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
    GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化PA11,PA12
	
	//CAN单元设置
   	CAN_InitStructure.CAN_TTCM=DISABLE;	//非时间触发通信模式   
  	CAN_InitStructure.CAN_ABOM=DISABLE;	//软件自动离线管理	  
  	CAN_InitStructure.CAN_AWUM=DISABLE;//睡眠模式通过软件唤醒(清除CAN->MCR的SLEEP位)
  	CAN_InitStructure.CAN_NART=ENABLE;	//使用报文自动传送 
  	CAN_InitStructure.CAN_RFLM=DISABLE;	//报文不锁定,新的覆盖旧的  
  	CAN_InitStructure.CAN_TXFP=DISABLE;	//优先级由报文标识符决定 
  	CAN_InitStructure.CAN_Mode= mode;	 //模式设置 
  	CAN_InitStructure.CAN_SJW=tsjw;	//重新同步跳跃宽度(Tsjw)为tsjw+1个时间单位 CAN_SJW_1tq~CAN_SJW_4tq
  	CAN_InitStructure.CAN_BS1=tbs1; //Tbs1范围CAN_BS1_1tq ~CAN_BS1_16tq
  	CAN_InitStructure.CAN_BS2=tbs2;//Tbs2范围CAN_BS2_1tq ~	CAN_BS2_8tq
  	CAN_InitStructure.CAN_Prescaler=brp;  //分频系数(Fdiv)为brp+1	
  	CAN_Init(CAN1, &CAN_InitStructure);   // 初始化CAN1
	
	//配置过滤器
 	CAN_FilterInitStructure.CAN_FilterNumber=0;	  //过滤器0
  	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; //标识符屏蔽位模式
  	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; //32位 
  	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;////32位ID
  	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;//32位ID
  	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;//32位MASK
  	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;//32位MASK
   	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//过滤器0关联到FIFO0
  	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; //激活过滤器0
  	CAN_FilterInit(&CAN_FilterInitStructure);//滤波器初始化


}


//can发送一组数据(固定格式:ID为0X12,标准帧,数据帧)	
//len:数据长度(最大为8)				     
//msg:数据指针,最大为8个字节.
//返回值:0,成功;
//		 其他,失败;
u8 CAN1_Send_Msg(u8* msg,u8 len)
{	
	u8 mbox;
	u16 i=0;
	CanTxMsg TxMessage;
	TxMessage.StdId=0x12;	 // 标准标识符为0
	TxMessage.ExtId=0x12;	 // 设置扩展标示符（29位）
	TxMessage.IDE=0;		  // 标识符类型（0为标准标识符，1为扩展标识符）
	TxMessage.RTR=0;		  // 消息类型为数据帧，一帧8位(为0是数据帧，为1是遥控帧)
	TxMessage.DLC=len;		// 0 ~ 8，标识当前 CAN 帧携带的数据字节数（CAN 总线单帧最多传 8 字节）
	for(i=0;i<len;i++)
		TxMessage.Data[i]=msg[i];				 // 第一帧信息          
	mbox= CAN_Transmit(CAN1, &TxMessage);   
	i=0;
	while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//等待发送结束
	if(i>=0XFFF)return 1;
	return 0;		
}

//can口接收数据查询
//buf:数据缓存区;	 
//返回值:0,无数据被收到;
//		 其他,接收的数据长度;
u8 CAN1_Receive_Msg(u8 *buf)
{		   		   
 	u32 i;
	CanRxMsg RxMessage;
    if( CAN_MessagePending(CAN1,CAN_FIFO0)==0)return 0;		//没有接收到数据,直接退出 
    CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);//读取数据	
    for(i=0;i<RxMessage.DLC;i++)
    buf[i]=RxMessage.Data[i];  
	return RxMessage.DLC;	
}


u8 buf[8]={0xA0,0xA1,0xA2,0xA3,0xA4,0xA5,0xA6,0xA7};
/*******************************************************************************
* 函 数 名         : main
* 函数功能		   : 主函数
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
int main()
{	
	u8 i=0,j=0;
	u8 key;
	u8 mode=0;
	u8 res;
	u8 tbuf[8];
	u8 rbuf[8];
	
	SysTick_Init(168);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  //中断优先级分组 分2组
	LED_Init();
	USART1_Init(115200);
	KEY_Init();
	CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_8tq,CAN_BS1_9tq,4,CAN_Mode_Normal);//500Kbps波特率
	
	while(1)
	{
		key=KEY_Scan(0);
		if(key==KEY_UP_PRESS)  //模式切换
		{
			mode=!mode;
			CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_8tq,CAN_BS1_9tq,4,mode);
			if(mode==0)
			{
				printf("Normal Mode\r\n");
			}
			else
			{
				printf("LoopBack Mode\r\n");
			}
		}
		if(key==KEY1_PRESS)  //发送数据
		{
			for(j=0;j<8;j++)
			{
				tbuf[j]=j;
			}
			res=CAN1_Send_Msg( buf ,8);
			if(res)
			{
				printf("Send Failed!\r\n");
			}
			else
			{
				printf("发送数据：");
				for(j=0;j<8;j++)
				{
					printf("%X  ",buf[j]);
				}
				printf("\r\n");
			}
		}
		res=CAN1_Receive_Msg(rbuf);
		if(res)
		{	
			printf("接收数据：");
			for(j=0;j<8;j++)
			{
				printf("%X  ",rbuf[j]);
			}

			printf("\r\n");
	        printf("接收数据长度：%d\r\n",res);

		}
		
		i++;
		if(i%20==0)
		{
			LED1=!LED1;
		}
		delay_ms(10);
	}
}
#elif defined(IAP_TEST)

int main()
{	
	u8 i=0;
	u8 key;
	
	//SCB->VTOR = FLASH_BASE | 0x10000;//设置偏移量(在system_stm32f4xx.c中设置)
	
	SysTick_Init(168);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  //中断优先级分组 分2组
	LED_Init();
	
	USART1_Init(115200);
	KEY_Init();
    printf("进入APP程序······\r\n");

	while(1)
	{

		i++;
		if(i%20==0)
		{
			LED1=!LED1;
			printf("1pw1\n");
			//printf("lpw2\n");
	
		}		
	 
		delay_ms(10);
	}
}






#else
#endif
