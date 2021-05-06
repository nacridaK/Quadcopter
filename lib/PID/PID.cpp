#include "PID.h"

PID::PID()
{
}

PID::PID(double ts, double kp, double ki, double kd, double tf, bool sat_mode, double sat_below, double sat_upper, bool clamp_mode)
{
	SetTs(ts);
	SetControllerGains(kp, ki, kd);
	SetTf(tf);
	SetSaturationMode(sat_mode);
	SetSaturationLimits(sat_below, sat_upper);
	SetClampingMode(clamp_mode);
}

void PID::Update(double err)
{
	error[0] = error[1];
	error[1] = err;
	CalculatePID();
	Saturation();
	Clamping();
}

double PID::GetOutput()
{
	return pid_sat;
}

void PID::CalculateP()
{ 
	P = Kp * error[1];
}

//void PID::CalculateI()		//I_ForwardEuler
//{
//	I_input = error[0];
//	I = I + !isClamped * Ki * Ts * I_input;
//}

void PID::CalculateI()		//I_BackwardEuler
{
	I_input = error[1];
	I = I + !isClamped * Ki * Ts * I_input;
}

//void PID::CalculateI()			//I_Trapezoidal
//{	
//	I_input = (error[1] + error[0]) / 2;
//	I = I + !isClamped * Ki * Ts *  I_input;
//}

//void PID::CalculateD()			//D_ForwardEuler
//{
//	D = (Kd * (error[1] - error[0]) + (Tf - Ts) * D) / Tf;
//}
//
void PID::CalculateD()			//D_BackwardEuler
{
	D = (Kd * (error[1] - error[0]) + Tf * D) / (Tf + Ts);
}

//void PID::CalculateD()			//D_Trapezoidal
//{
//	D = (2 * Kd * (error[1] - error[0]) + (2 * Tf - Ts) * D) / (2 * Tf + Ts);
//}

void PID::CalculatePID()
{
	CalculateP();
	CalculateI();
	CalculateD();
	pid = P + I + D;
}

void PID::Saturation()
{
	if (saturation_mode)
	{
		if (pid < sat_limits[0])
		{
			pid_sat = sat_limits[0];
			isSaturated = true;
		}
		else if (pid > sat_limits[1])
		{
			pid_sat = sat_limits[1];
			isSaturated = true;
		}
	}
	else
	{
		pid_sat = pid;
		isSaturated = false;
	}
}

void PID::Clamping()
{
	if (saturation_mode && clamping_mode)
	{
		signCheck = sign(pid) == sign(I_input);
		isClamped = isSaturated && signCheck;
	}
	else
		isClamped = false;
}

uint8_t PID::sign(double x)
{
	if (x > 0)
		return 1;
	else if (x < 0)
		return -1;
	return 0;
}

void PID::SetTs(double ts)
{
	Ts = ts;
}

void PID::SetKpGain(double kp)
{
	Kp = kp;
}

void PID::SetKiGain(double ki)
{
	Ki = ki;
}

void PID::SetKdGain(double kd)
{
	Kd = kd;
}

void PID::SetControllerGains(double kp, double ki, double kd)
{
	SetKpGain(kp);
	SetKiGain(ki);
	SetKdGain(kd);
}

void PID::SetTf(double tf)
{
	Tf = tf;
}

void PID::SetSaturationMode(bool sat_mode)
{
	saturation_mode = sat_mode;
}

void PID::SetClampingMode(bool clamp_mode)
{
	clamping_mode = clamp_mode;
}

void PID::SetBelowLimit(double sat_below)
{
	sat_limits[0] = sat_below;
}

void PID::SetUpperLimit(double sat_upper)
{
	sat_limits[1] = sat_upper;
}

void PID::SetSaturationLimits(double sat_below, double sat_upper)
{
	SetBelowLimit(sat_below);
	SetUpperLimit(sat_upper);
}