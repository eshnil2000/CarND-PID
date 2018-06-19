#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	Kp_=Kp;
	Ki_=Ki;
	Kd_=Kd;
	p_error=0;
	i_error=0;
	d_error=0;
	cte_prev_ = 0;
	cte_ = 0;
	cte_mem_ = 0;
}

void PID::UpdateError(double cte) {
	//Initial value of p_error to set u differential error d_error
	cte_prev_ = cte_;
	cte_ = cte;
	p_error = cte_;
	d_error = cte_-cte_prev_;
    i_error += cte;
    

}

double PID::TotalError() {
	return Kp_* p_error+ Ki_* i_error + Kd_* d_error;
	//return 0.5;
}

