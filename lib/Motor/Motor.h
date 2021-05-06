#include <inttypes.h>

class Motor
{
private:
	const uint8_t PoleCount;
	const uint16_t DShotMAX = 2047, DShotMIN = 48;
	uint16_t SpeedDShot, SpeedRPM;
public:
	Motor(uint8_t poleCount = 0);
	uint16_t GetSpeedDShot();
	void SetSpeedDShot(int32_t DesiredSpeedDShot);
	uint16_t GetSpeedRPM();
	void SetSpeedRPM(uint16_t speedRPM);
};