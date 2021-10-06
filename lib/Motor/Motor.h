#include <inttypes.h>

class Motor
{
private:
	const uint8_t KutupSayisi;
	const uint16_t DShotMAX = 200, DShotMIN = 0;
	//2047 - 48
	uint16_t HizDShot, HizRPM;
public:
	Motor(uint8_t poleCount = 0);
	uint16_t GetHizDShot();
	void SetHizDShot(int32_t IstenenHizDShot);
	uint16_t GetHizRPM();
	void SetHizRPM(uint16_t speedRPM);
};