#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	// initialize gains and errors
	this->Kp = Kp;
	this->Ki = Ki;
	this->Kd = Kd;

	p_error = 0.0;
	d_error = 0.0;
	i_error = 0.0;

}

void PID::UpdateError(double cte) {
		// d error
		d_error = cte - p_error;
		
		// p error
		p_error = cte;
		// i error
		i_error += cte;

}

double PID::TotalError() {

	double control_action;
	// controller
	control_action = -Kp * p_error - Kd * d_error - Ki * i_error;

return control_action;

}

