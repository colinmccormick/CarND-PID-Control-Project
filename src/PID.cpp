#include "PID.h"

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_set, double Ki_set, double Kd_set) {

	// Initialize the controller with appropriate constants
	Kp = Kp_set;
	Ki = Ki_set;
	Kd = Kd_set;

	// Initialize errors
	p_error = 0;
	d_error = 0;
	i_error = 0;

	// Initialize tuning params
	n_frames = 0;
	cte_squared = 0;

}

void PID::UpdateError(double cte) {

	// Calculate P, I and D errors
	d_error = cte - p_error;
	i_error += cte;
	p_error = cte;
}

double PID::TotalError() {

	double totalError = Kp * p_error + Kd * d_error + Ki * i_error;
	return totalError;
}

double PID::CalculateSteering() {

	double steering = -1 * TotalError();

	// Clip steering angle
	steering = steering > 1 ? 1 : steering;
	steering = steering < -1 ? -1 : steering;

	return steering;
}

