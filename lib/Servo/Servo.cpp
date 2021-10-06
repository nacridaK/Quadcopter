#include "main.h"
#include "Servo.h"

Servo::Servo(iBus* Ibus, uint8_t kanal, float olcek, float offset)
{
	ibus = Ibus;
	Kanal = kanal;
	Olcek = olcek;
	Offset = offset;
}

float Servo::ChannelValue()
{
	return (ibus->KanalVerisi(Kanal) + Offset) * Olcek;
}