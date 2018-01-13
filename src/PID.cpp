#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	this->Kp_ = Kp;
	this->Ki_ = Ki;
	this->Kd_ = Kd;

	this->p_error_ = 0;
	this->i_error_ = 0;
	this->d_error_ = 0;
}

void PID::UpdateError(double cte) {
	static double pre_cte = 0;
	double delta_cte = cte - pre_cte;
	p_error_  = cte;
	i_error_ += cte;
	d_error_  = delta_cte;
	pre_cte = cte;
}

double PID::TotalError() {
	double ret = -Kp_*p_error_ - Ki_*i_error_ - Kd_*d_error_;
	//ret *= -1;
	return ret;
}

