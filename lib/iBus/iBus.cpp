#include "main.h"
#include "iBus.h"

iBus::iBus(UART_HandleTypeDef* huart, TIM_HandleTypeDef* htim, uint32_t RX_channel, uint32_t IDLE_channel, uint32_t NRX_channel)
{
	Huart = huart;
	Htim = htim;
	HAL_TIM_IC_Start_IT(Htim, RX_channel);
	HAL_TIM_OC_Start_IT(Htim, IDLE_channel);
	HAL_TIM_OC_Start_IT(Htim, NRX_channel);
}

void iBus::RXComplete()
{
	ComputeChecksum();
	Checksum_rx =  (Buffer[31] << 8) | Buffer[30];
	if (Checksum_cc == Checksum_rx)
	{
		CRCerror = 0;
		for (uint8_t i = 0; i < 20; i += 2)
			Channel[i / 2] = (Buffer[i + 3] << 8) | Buffer[i + 2];
	}
	else
	{
		CRCerror = 1;
	}
}

const uint16_t* const iBus::ReturnChannelPointer(uint8_t channelNo)
{
	return Channel + channelNo;
}

void iBus::ComputeChecksum()
{
	Checksum_cc = 0xFFFF;
	for (uint8_t i = 0; i < 30; i++)
		Checksum_cc -= Buffer[i];
}

void iBus::Receiving()
{
	if(!IsReceiving)
	{
		IsReceiving = 1;
	}
}

void iBus::Idle()
{
	if(IsReceiving)
	{
		HAL_UART_Receive_DMA(Huart, Buffer, 32);
	}
}

void iBus::NotReceiving()
{
	if(IsReceiving)
	{
		IsReceiving = 0;
		HAL_UART_DMAStop(Huart);
	}
}