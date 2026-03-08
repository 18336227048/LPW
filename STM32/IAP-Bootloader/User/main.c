#include "system.h"
#include "SysTick.h"
#include "led.h"
#include "usart.h"
#include "tftlcd.h"
#include "key.h"
#include "stm32_flash.h"
#include "iap.h"

/*******************************************************************************
 * 函 数 名         : main
 * 函数功能		   : 主函数
 * 输    入         : 无
 * 输    出         : 无
 *******************************************************************************/
int main()
{
	u8 t;
	u8 key;
	u16 oldcount = 0; // 老的串口接收数据值
	u32 applenth = 0; // 接收到的app代码长度
	

	SysTick_Init(168);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); // 中断优先级分组 分2组
	LED_Init();
	USART1_Init(115200);
	KEY_Init();
    printf("进入bootloader模式!\r\n	");
	while (1)
	{
		if (USART1_RX_CNT)
		{
			if (oldcount == USART1_RX_CNT) // 新周期内,没有收到任何数据,认为本次数据接收完成.
			{
				applenth = USART1_RX_CNT;
				oldcount = 0;
				USART1_RX_CNT = 0;
				printf("用户程序接收完成!\r\n");
				printf("代码长度:%dBytes\r\n", applenth);
			}
			else
				oldcount = USART1_RX_CNT;
		}

		t++;
		delay_ms(10);//没10ms轮询一次判断上述条件语句
		if (t == 30)
		{
			LED1 = !LED1;
			t = 0;
		}
		key = KEY_Scan(0);
		if (key == KEY_UP_PRESS)
		{
			if (applenth)
			{
				printf("开始更新固件...\r\n");

				if (((*(vu32 *)(0X20001000 + 4)) & 0xFF000000) == 0x08000000) // 判断是否为0X08XXXXXX.
				{
					iap_write_appbin(FLASH_APP2_ADDR, USART1_RX_BUF, applenth); // 更新FLASH代码

					printf("固件更新完成!\r\n");
				}
				else
				{

					printf("非FLASH应用程序!\r\n");
				}
			}
			else
			{
				printf("没有可以更新的固件!\r\n");
			}
		}

		if (key == KEY2_PRESS)
		{
			if (applenth)
			{
				printf("固件清除完成!\r\n");

				applenth = 0;
			}
			else
			{
				printf("没有可以清除的固件!\r\n");
			}
		}
		if (key == KEY1_PRESS)
		{
			printf("开始执行FLASH用户代码!!\n");
			if (((*(vu32 *)(FLASH_APP1_ADDR + 4)) & 0xFF000000) == 0x08000000) // 判断是否为0X08XXXXXX.
			{
				iap_load_app(FLASH_APP1_ADDR); // 执行FLASH APP代码
			}
			else
			{
				printf("非FLASH应用程序,无法执行!\r\n");
			}
		}
		if(key == KEY0_PRESS)//KEY0按下,则进入固件升级模式
		{
			STM32_FLASH_Write(FLASH_APP1_ADDR,(u32 *)FLASH_APP2_ADDR,applenth); // 将第二个应用程序拷贝到第一个应用程序的位置(FLASH_APP1_
			printf("固件大小：%d\r\n",applenth);
			printf("固件拷贝到app1完成!\r\n");
		}
	}
}
