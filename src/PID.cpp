#include "PID.h"
#include <algorithm>


PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_)
{
  // TODO: Initialize PID coefficients (and errors, if needed)
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;

  // iteration
  iter = 0;

}

void PID::UpdateError(double cte)
{
  // TODO: Update PID errors based on cte.

  d_error = cte-p_error;
  p_error = cte;
  i_error += cte;

  iter++;
}

double PID::TotalError()
{
  // TODO: Calculate and return the total error
  
  double error = (-Kp * p_error) - (Kd * d_error) - (Ki * i_error);

  return error;// TODO: Add your total error calc here!
}