#include <stdint.h>

struct dshotProtocolControl
{
	uint16_t value;
	bool requestTelemetry;
};

class DShot
{
	private:
		dshotProtocolControl pcb;
	  const uint32_t PWM_const;
		uint16_t wait;
		uint16_t repeat;
		uint32_t bitToPWM(bool bit);
		unsigned computeChecksum(uint16_t packet);
		uint16_t prepareDshotPacket(dshotProtocolControl *pcb);
		void convertDshotPacketToPWM_Array(uint16_t packet);
	
	public:
		DShot(uint32_t ARR);
		uint32_t PWM_Array[32];
		void update(uint16_t value = 0, bool requestTelemetry = false);
		void setWait(uint16_t _wait);
		void setRepeat(uint16_t _repeat);
};