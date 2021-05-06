#include "Motor.h"

Motor::Motor(uint8_t poleCount) : PoleCount(poleCount)
{
}

uint16_t Motor::GetSpeedDShot()
{
	return SpeedDShot;
}

void Motor::SetSpeedDShot(int32_t DesiredSpeedDShot)
{
	if (DesiredSpeedDShot > DShotMAX)
		SpeedDShot = DShotMAX;
	else if (DesiredSpeedDShot < DShotMIN)
		SpeedDShot = DShotMIN;
	else
		SpeedDShot = DesiredSpeedDShot;
}

uint16_t Motor::GetSpeedRPM()
{
	return SpeedRPM;
}

void Motor::SetSpeedRPM(uint16_t ERPM)
{
	//Buraya ERPM -> RPM dönüþümü eklenecek. Telemetri PDF'inde bilgilendirme var.
}