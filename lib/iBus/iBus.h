#pragma once
#include <stm32f4xx_hal_uart.h>
#include <stm32f4xx_hal_tim.h>
#include <stm32f4xx_hal.h>

class iBus
{
private:
	UART_HandleTypeDef* Huart;
	TIM_HandleTypeDef* Htim;
	uint32_t RX_Kanal, IDLE_Kanal, NRX_Kanal;
	uint16_t Kanal[10], Checksum_CC, Checksum_RX;
	uint8_t AraBellek[32], GeliyorMu, CRC_Hata;
	void ChecksumHesapla();

public:
	iBus(UART_HandleTypeDef* huart, TIM_HandleTypeDef* htim, uint32_t rx_Channel, uint32_t idle_Channel, uint32_t nrx_Channel);
	void Basla();
	void Dur();
	void Geliyor();
	void Beklemede();
	void Gelmiyor();
  void PaketGeldi();
	uint16_t KanalVerisi(uint8_t kanal);
};