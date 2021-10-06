#include "iBus/iBus.h"

class Servo
{
private:
	iBus* ibus;
	uint8_t Kanal;
	float Olcek;
	float Offset;
public:
	Servo(iBus* ibus, uint8_t channelNo, float Scale = 1, float Offset = -1000);
	float ChannelValue();
};