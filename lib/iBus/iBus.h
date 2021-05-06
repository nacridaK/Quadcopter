#pragma once
#include <stm32f4xx_hal_uart.h>
#include <stm32f4xx_hal_tim.h>
#include <stm32f4xx_hal.h>

class iBus
{
private:
	UART_HandleTypeDef* Huart;
	TIM_HandleTypeDef* Htim;
	uint16_t Channel[10], Checksum_cc, Checksum_rx;
	uint8_t Buffer[32], IsReceiving, CRCerror;
	void ComputeChecksum();

public:
	iBus(UART_HandleTypeDef* huart, TIM_HandleTypeDef* htim, uint32_t RX_channel, uint32_t IDLE_channel, uint32_t NRX_channel);
	void Start();
	void Receiving();
	void Idle();
	void NotReceiving();
  void RXComplete();
	const uint16_t* const ReturnChannelPointer(uint8_t channel);
};