#include "main.h"
#include "iBus.h"

iBus::iBus(UART_HandleTypeDef* huart, TIM_HandleTypeDef* htim, uint32_t rx_Channel, uint32_t idle_Channel, uint32_t nrx_Channel)
{
	Huart = huart;
	Htim = htim;
	RX_Kanal = rx_Channel;
	IDLE_Kanal = idle_Channel;
	NRX_Kanal = nrx_Channel;
}

void iBus::Basla()
{
	HAL_TIM_IC_Start_IT(Htim, RX_Kanal);
	HAL_TIM_OC_Start_IT(Htim, IDLE_Kanal);
	HAL_TIM_OC_Start_IT(Htim, NRX_Kanal);
}

void iBus::Dur()
{
	HAL_TIM_IC_Stop_IT(Htim, RX_Kanal);
	HAL_TIM_OC_Stop_IT(Htim, IDLE_Kanal);
	HAL_TIM_OC_Stop_IT(Htim, NRX_Kanal);
}

void iBus::PaketGeldi()
{
	ChecksumHesapla();
	Checksum_RX =  (AraBellek[31] << 8) | AraBellek[30];
	if (Checksum_CC == Checksum_RX)
	{
		CRC_Hata = 0;
		for (uint8_t i = 0; i < 20; i += 2)
			Kanal[i / 2] = (AraBellek[i + 3] << 8) | AraBellek[i + 2];
	}
	else
		CRC_Hata = 1;
}

void iBus::ChecksumHesapla()
{
	Checksum_CC = 0xFFFF;
	for (uint8_t i = 0; i < 30; i++)
		Checksum_CC -= AraBellek[i];
}

void iBus::Geliyor()
{
	if(!GeliyorMu)
		GeliyorMu = 1;
}

void iBus::Beklemede()
{
	if(GeliyorMu)
		HAL_UART_Receive_DMA(Huart, AraBellek, 32);
}

void iBus::Gelmiyor()
{
	if(GeliyorMu)
	{
		GeliyorMu = 0;
		HAL_UART_DMAStop(Huart);
	}
}

uint16_t iBus::KanalVerisi(uint8_t kanal)
{
	if (kanal < 10)
		return Kanal[kanal];
	return 0;
}