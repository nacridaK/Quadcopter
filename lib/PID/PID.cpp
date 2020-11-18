#include "PID.h"

PID::PID(double ts, double kp, double ki, double kd, double tf)
{
	Ts = ts;
	Kp = kp;
	Ki = ki;
	Kd = kd;
	Tf = tf;
}

double PID::update(double err)
{
	error[1] = err;
	CalculateC();
	Saturation();
	Clamping();
	error[0] = error[1];
	return C_sat;

}

void PID::CalculateP()
{ 
	P = Kp * error[1];
}

//void PID::CalculateI()		//I_ForwardEuler
//{
//	I_input = !isClamped * error[0];
//	I = I + Ki * Ts * I_input;
//}

//void PID::CalculateI()		//I_BackwardEuler
//{
//	I_input = !isClamped * error[1];
//	I = I + Ki * Ts * I_input;
//}

void PID::CalculateI()			//I_Trapezoidal
{	
	I_input = !isClamped * (error[1] + error[0]) / 2;
	I = I + Ki * Ts *  I_input;
}

//void PID::CalculateD()			//D_ForwardEuler
//{
//	D = (Kd * (error[1] - error[0]) + (Tf - Ts) * D) / Tf;
//}
//
//void PID::CalculateD()			//D_BackwardEuler
//{
//	D = (Kd * (error[1] - error[0]) + Tf * D) / (Tf + Ts);
//}

void PID::CalculateD()			//D_Trapezoidal
{
	D = (2 * Kd * (error[1] - error[0]) + (2 * Tf - Ts) * D) / (2 * Tf + Ts);
}

void PID::CalculateC()
{
	CalculateP();
	CalculateI();
	CalculateD();
	C = P + I + D;
}

void PID::Saturation()
{
	if (saturation_mode)
	{
		if (C < sat_limits[0])
		{
			C_sat = sat_limits[0];
			isSaturated = true;
		}
		else if (C > sat_limits[1])
		{
			C_sat = sat_limits[1];
			isSaturated = true;
		}
	}
	else
	{
		C_sat = C;
		isSaturated = false;
	}
}

void PID::Clamping()
{
	if (saturation_mode && clamping_mode)
	{
		signCheck = sign(C) == sign(I_input);
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