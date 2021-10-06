#include "Motor.h"

Motor::Motor(uint8_t kutupSayisi) : KutupSayisi(kutupSayisi)
{
}

uint16_t Motor::GetHizDShot()
{
	return HizDShot;
}

void Motor::SetHizDShot(int32_t IstenenHizDShot)
{
	if (IstenenHizDShot > DShotMAX)
		HizDShot = DShotMAX;
	else if (IstenenHizDShot < DShotMIN)
		HizDShot = DShotMIN;
	else
		HizDShot = IstenenHizDShot;
}

uint16_t Motor::GetHizRPM()
{
	return HizRPM;
}

void Motor::SetHizRPM(uint16_t ERPM)
{
	//Buraya ERPM -> RPM dönüsümü eklenecek. Telemetri PDF'inde bilgilendirme var.
}