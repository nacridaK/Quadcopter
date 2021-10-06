#include "DShot.h"
#include <stm32f4xx_hal_tim.h>

DShot::DShot(TIM_HandleTypeDef* htim, uint8_t channel)
{
	Htim = htim;
	Channel = channel;
}

uint32_t DShot::bitToPWM(bool bit)
{
	return PWM_const << bit;
}

HAL_StatusTypeDef DShot::Basla()
{
	PWM_const = .375 * (Htim->Init.Period + 1);
	for (uint8_t i = 0; i < 16; i++)
		PWM_Array[i] = PWM_const;
	
	HAL_StatusTypeDef ret = HAL_TIM_PWM_Start_DMA(Htim, Channel, PWM_Array, LENGTH);
	Htim->State = HAL_TIM_STATE_READY;
	return ret;
}

HAL_StatusTypeDef DShot::Dur()
{
	return HAL_TIM_PWM_Stop_DMA(Htim, Channel);
}

unsigned DShot::computeChecksum(uint16_t packet)
{
	// compute checksum
  unsigned csum = 0;
  unsigned csum_data = packet;
  for (uint8_t i = 0; i < 3; i++)
	{
		csum ^=  csum_data;   // xor data by nibbles
		csum_data >>= 4;
  }
  // append checksum
  csum &= 0xf;
	return csum;
}

uint16_t DShot::prepareDshotPacket(dshotProtocolControl *pcb)
{
	uint16_t packet = (pcb->value << 1) | (pcb->requestTelemetry ? 1 : 0);
  pcb->requestTelemetry = false;    // reset telemetry request to make sure it's triggered only once in a row
  unsigned csum = computeChecksum(packet);
  packet = (packet << 4) | csum;
  return packet;
}

void DShot::convertDshotPacketToPWM_Array(uint16_t packet)
{
//	for (uint8_t i = 0; i < 16; i++)
//		PWM_Array[15 - i] = bitToPWM((packet >> i) & 1);
	if (!(repeat))
	{	
		if (wait)
		{
			for (uint8_t i = 0; i < 16; i++)
				PWM_Array[15 - i] = 0;
			wait--;
		}
		else
			for (uint8_t i = 0; i < 16; i++)
				PWM_Array[15 - i] = bitToPWM((packet >> i) & 1);
	}
	else
		repeat--;
}

void DShot::update(uint16_t value, bool requestTelemetry)
{
	if ((pcb.value != value) || (pcb.requestTelemetry != requestTelemetry))
	{
		pcb.value = value;
		pcb.requestTelemetry = requestTelemetry;
		convertDshotPacketToPWM_Array(prepareDshotPacket(&pcb));		
	}
}

void DShot::setWait(uint16_t _wait)
{
	if (!wait)
		wait = _wait;
}

void DShot::setRepeat(uint16_t _repeat)
{
	if (!repeat)
		repeat = _repeat - 1;
}