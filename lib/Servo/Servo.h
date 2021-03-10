#include "iBus/iBus.h"

class Servo
{
private:
	const uint16_t* Channel_Raw;
public:
	Servo(iBus* ibus, uint8_t channelNo, float Scale = 1, float Offset = -1000);
	float ChannelValue();
	float Scale;
	float Offset;
};