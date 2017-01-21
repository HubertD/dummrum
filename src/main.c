#include "stm32f0xx_hal.h"
#include "can.h"

enum {
	CAN_HEARTBEAT_INTERVAL = 1000,
	CANID_HEARTBEAT  = 0x80102000,
	CANID_ACTIVATE   = 0x80102010,
	CANID_DEACTIVATE = 0x80102011,
	CANID_RESET      = 0x801020FF,
};

void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);

static uint32_t drl_active_until = 0;
void drl_activate(can_msg_t *msg);
void drl_send_heartbeat_if_needed(can_data_t *can, uint8_t status);

int main(void)
{
	HAL_Init();
	SystemClock_Config();
	MX_GPIO_Init();
	HAL_GPIO_WritePin(GPIOA, LED_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, DUMMRUM_Pin, GPIO_PIN_RESET);

	can_data_t can0;
	can_msg_t msg;

	can_init(&can0, CAN);
	can_set_bittiming(&can0, 6, 13, 2, 1);
	can_enable(&can0, false, false, false);
	HAL_Delay(10);

	while (1)
	{
		uint8_t drl_status = (drl_active_until >= HAL_GetTick()) ? 1 : 0;
		if (drl_status)
		{
			HAL_GPIO_WritePin(GPIOA, LED_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, DUMMRUM_Pin, GPIO_PIN_SET);
		} else {
			HAL_GPIO_WritePin(GPIOA, LED_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA, DUMMRUM_Pin, GPIO_PIN_RESET);
		}

		drl_send_heartbeat_if_needed(&can0, drl_status);

		if (can_receive(&can0, &msg))
		{
			switch (msg.id)
			{
				case CANID_ACTIVATE:
					drl_activate(&msg);
					break;
				case CANID_DEACTIVATE:
					drl_active_until = 0;
					break;
			}
		}
	}
}

void drl_send_heartbeat_if_needed(can_data_t *can, uint8_t status)
{
	static uint32_t t_next = 0;
	uint32_t t = HAL_GetTick();
	if (t > t_next)
	{
		can_msg_t msg;
		msg.id = CANID_HEARTBEAT;
		msg.dlc = 5;
		msg.data[0] = (t >> 0)  & 0xFF;
		msg.data[1] = (t >> 8)  & 0xFF;
		msg.data[2] = (t >> 16) & 0xFF;
		msg.data[3] = (t >> 24) & 0xFF;
		msg.data[4] = status;
		can_send(can, &msg);
		t_next += CAN_HEARTBEAT_INTERVAL;
	}
}

void drl_activate(can_msg_t *msg)
{
	uint32_t t = ((uint32_t)msg->data[0] << 24) |
				 ((uint32_t)msg->data[1] << 16) |
				 ((uint32_t)msg->data[2] <<  8) |
				 ((uint32_t)msg->data[3] << 0);

	if (t==0)
	{
		drl_active_until = 0xFFFFFFFF;
	}
	else
	{
		drl_active_until = HAL_GetTick() + t;
	}
}

void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;

	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
	RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
							  |RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
	{
		Error_Handler();
	}

	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	HAL_GPIO_WritePin(GPIOA, LED_Pin|DUMMRUM_Pin, GPIO_PIN_RESET);
	GPIO_InitStruct.Pin = LED_Pin|DUMMRUM_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void Error_Handler(void)
{
	while(1)
	{
	}
}
