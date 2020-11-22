// https://www.youtube.com/watch?v=NVLXCwc8HzM&list=PLn8PRpmsu08pQBgjxYFXSsODEF3Jqmm-y&index=2
#include <inttypes.h>
class PID
{
private:
	double error[2] = {0};
	double Kp, Ki, Kd, Tf, Ts;
	double P, I, D, pid, pid_sat, I_input;
	double sat_limits[2];
	bool isSaturated, saturation_mode, isClamped, clamping_mode, signCheck;
	void CalculateP();
	void CalculateI();
	void CalculateD();
	void CalculatePID();
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
	PID();
	PID(double ts, double kp, double ki, double kd, double tf, bool sat_mode, double sat_below, double sat_upper, bool clamp_mode);
	double Update(double err);
	void SetTs(double ts);
	void SetKpGain(double kp);
	void SetKiGain(double ki);
	void SetKdGain(double kd);
	void SetControllerGains(double kp, double ki, double kd);
	void SetTf(double tf);
	void SetSaturationMode(bool sat_mode);
	void SetClampingMode(bool clamp_mode);
	void SetBelowLimit(double sat_below);
	void SetUpperLimit(double sat_upper);
	void SetSaturationLimits(double sat_below, double sat_upper);
};