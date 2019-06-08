#include "PID.h"

#include <iostream>
#include <algorithm>

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
  K[K_P] = Kp_;
  K[K_I] = Ki_;
  K[K_D] = Kd_;

  std::cout << "Init with " << Kp_ << ", " << Ki_ << ", " << Kd_ << std::endl;

  for(int i = 0; i < K_NoOf; ++i)
    error[i] = 0;
}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
   error[K_D] = cte - error[K_P];
   error[K_P] = cte;
   error[K_I] += cte;
}

double PID::TotalError() {
  double total = 0.;

  for(int i = 0; i < K_NoOf; ++i)
    total += K[i] * error[i];

  return total;
}

double PID::Control(double lower_limit, double upper_limit)
{
  double control_value = 0.;

  for(int i = 0; i < K_NoOf; ++i)
    control_value -= K[i] * error[i];

  // handle max values for control value
  control_value = std::max(lower_limit, control_value);
  control_value = std::min(upper_limit, control_value);

  return control_value;
}

void PID::getCoefficients(double coeffs[K_NoOf])
{
  for(int i = 0; i < K_NoOf; ++i)
    coeffs[i] = K[i];
}
