// https://www.youtube.com/watch?v=NVLXCwc8HzM&list=PLn8PRpmsu08pQBgjxYFXSsODEF3Jqmm-y&index=2
#include <inttypes.h>
class PID
{
private:
	double error[2] = {0};
	double Kp, Ki, Kd, Tf, Ts;
	double P, I, D, C, C_sat, I_input;
	double sat_limits[2];
	bool isSaturated, isClamped, signCheck;
	void CalculateP();
	void CalculateI();
	void CalculateD();
	void CalculateC();
	void Saturation();
	void Clamping();
	uint8_t sign(double x);
	//void I_ForwardEuler();
	//void I_BackwardEuler();
	//void I_Trapezoidal();
	//void D_ForwardEuler();
	//void D_BackwardEuler();
	//void D_Trapezoidal();
public:
	PID(double ts, double kp, double ki, double kd, double tf);
	bool saturation_mode, clamping_mode;
	double update(double err);
};