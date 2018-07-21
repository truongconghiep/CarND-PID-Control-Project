#include "PID.h"
#include <string.h>
#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) 
{
	PID::Kp = Kp;
	PID::Ki = Ki;
	PID::Kd = Kd;
	PID::p_error = 0;
	PID::i_error = 0;
	PID::d_error = 0;
}

void PID::UpdateError(double cte) 
{
	PID::d_error = cte - PID::p_error;
	PID::p_error = cte;
	PID::i_error += cte;
}

double PID::TotalError() 
{ 
	return (-PID::Kp * PID::p_error - PID::Kd * PID::d_error - PID::Ki * PID::i_error);
}

void PID::TwiddleInit(int RunningSteps, TwiddleState_ten InitState, double Tolerant)
{
	int Counter_i;
	TwiddleState = InitState;
	LastTwiddleState = InitState;
	Tolerant = Tolerant;
	TwiddleRunningSteps = RunningSteps;
	for (Counter_i = 0; Counter_i < 3; Counter_i++)
	{
		dk[Counter_i] = 1.0;
		K[Counter_i] = 0.0;
	}
	TwiddleRunningCounter = 0;
	TwiddleBestErr = 0.0;
	TwiddleLastErr = 0.0;
	AttrCounter = 0;
}

bool PID::IsTwiddleNeeded()
{
	return (TwiddleState != DONE) && (LastTwiddleState != DONE);
}

bool PID::Twiddle(float cte)
{
	bool RetVal_b = false;

	switch (TwiddleState)
	{
	case RUN:
		TwiddleRunningState(cte);
		break;
	case PROCEED_1:
		RetVal_b = TwiddleProceed1();
		break;
	case PROCEED_2:
		RetVal_b = TwiddleProceed2();
		break;
	case PROCEED_3:
		TwiddleProceed3();
		break;
	default:
		break;
	}

	return RetVal_b;
}

void PID::TwiddleRunningState(float cte)
{
	if (TwiddleRunningCounter < TwiddleRunningSteps)
	{
		TwiddleLastErr += cte * cte;
		TwiddleRunningCounter++;
	}
	else
	{
		TwiddleLastErr = TwiddleLastErr / TwiddleRunningSteps;
		TwiddleRunningCounter = 0;

		if (LastTwiddleState == RUN)
		{
			TwiddleBestErr = TwiddleLastErr;
			TwiddleState = PROCEED_1;
			LastTwiddleState = PROCEED_1;

		}
		else if (LastTwiddleState == PROCEED_1)
		{
			TwiddleState = PROCEED_2;
			LastTwiddleState = PROCEED_2;
		}
		else if (LastTwiddleState == PROCEED_2)
		{
			TwiddleState = PROCEED_3;
			LastTwiddleState = PROCEED_3;
		}
	}

}

bool PID::TwiddleProceed1()
{
	bool RetVal = false;
	if (TwiddleSumDiff() > Tolerant)
	{
		if (AttrCounter < 3)
		{
			K[AttrCounter] += dk[AttrCounter];
			TwiddleState = RUN;
			LastTwiddleState = PROCEED_1;
			RetVal = true;
			TwiddleLastErr = 0.0;
		}
		else
		{
			AttrCounter = 0;
		}
	}
	else
	{
		TwiddleState = DONE;
		LastTwiddleState = DONE;
		AttrCounter = 0;
		std::cout << "K0 " << K[0] << " K1 " << K[1] << " K2 " << K[2] << endl;

	}
	return RetVal;
}

bool PID::TwiddleProceed2()
{
	bool RetVal = false;
	if (TwiddleLastErr < TwiddleBestErr)
	{
		TwiddleBestErr = TwiddleLastErr;
		dk[AttrCounter] *= 1.1;
		AttrCounter++;
		TwiddleState = PROCEED_1;
		LastTwiddleState = PROCEED_1;
	}
	else
	{
		K[AttrCounter] -= 2 * dk[AttrCounter];
		TwiddleState = RUN;
		TwiddleLastErr = 0.0;
		LastTwiddleState = PROCEED_2;
		RetVal = true;
	}
	return RetVal;
}

void PID::TwiddleProceed3()
{
	if (TwiddleLastErr < TwiddleBestErr)
	{
		TwiddleBestErr = TwiddleLastErr;
		dk[AttrCounter] *= 1.1;
	}
	else
	{
		K[AttrCounter] += dk[AttrCounter];
		dk[AttrCounter] *= 0.9;
	}
	AttrCounter++;
	TwiddleState = PROCEED_1;
	LastTwiddleState = PROCEED_1;
}

float PID::TwiddleSumDiff()
{
	return dk[0] + dk[1] + dk[2];
}