#include "stm32f10x.h"                  // Device header
#include "Delay.h"

static volatile uint8_t Key_Num;

void Key_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

}

uint8_t Key_GetNum(void)
{
	uint8_t Temp;

	__disable_irq();
	Temp = Key_Num;
	Key_Num = 0;
	__enable_irq();
	return Temp;
}

uint8_t Key_GetState(void)
{
	if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_14) == 0)
	{
		return 1;
	}
	if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_15) == 0)
	{
		return 2;
	}
	return 0;
}

void Key_Tick(void)
{
	static uint8_t RawState = 0;
	static uint8_t StableState = 0;
	static uint8_t StableCount = 0;
	uint8_t NewState = Key_GetState();

	if (NewState != RawState)
	{
		RawState = NewState;
		StableCount = 0;
	}
	else if (StableCount < 20)
	{
		StableCount++;
	}

	if (StableCount >= 20 && StableState != RawState)
	{
		if (StableState != 0 && RawState == 0)
			Key_Num = StableState;
		StableState = RawState;
	}
}
