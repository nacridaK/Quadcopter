#include "main.h"
#include "Servo.h"

Servo::Servo(iBus* ibus, uint8_t channelNo, float scale, float offset)
{
	Channel_Raw = ibus->ReturnChannelPointer(channelNo);
	Scale = scale;
	Offset = offset;
}

float Servo::ChannelValue()
{
	return (*Channel_Raw + Offset) * Scale;
}