#include <iostream>
#include <cmath>
#include <limits>

#include "PIDAutotune.h"

PIDAutotune::PIDAutotune(PID* cont)
  : current_state(Start)
  , controller(cont)
  , current_steps(0)
  , total_cte(0)
  , best_error(std::numeric_limits<double>::max())
  , current_tune_parameter(0)
  , was_off_track(false)
  , round(0)
{
  controller->getCoefficients(&p[0]);

  std::cout << "Current params: ";

  for(int i = 0; i < PID::K_NoOf; ++i) {
    dp[i] = .5;
    std::cout << p[i] << " ";
  }

  dp[0] = 0.2;

  std::cout << std::endl;
}

bool PIDAutotune::didFinishRun()
{
  current_steps++;

  ///std::cout << "Step: " << current_steps << std::endl;

  // take about 2000 timesteps at 30mph to collect enough data for optimization
  //return (current_steps > 1000);  // around 1 round at 30mph
  return (current_steps > 250);  // around 1 round at 30mph
}

void PIDAutotune::addUpError(double cte)
{
  if(current_steps > init_steps) {
    total_cte += cte;
  }
}

bool PIDAutotune::cteIndicatesOffTrack(double cte)
{
  if((current_steps > init_steps) && (std::fabs(cte) > 6.)) {
    was_off_track = true;
    return true;
  } else {
    return false;
  }
}

bool PIDAutotune::twiddle(double tolerance)
{
  // DEBUG
  std::cout << "Round " << round << " twiddle" << std::endl;
  round++;
  std::cout << "Current parameter: " << current_tune_parameter << std::endl;
  std::cout << "Current state: ";
  switch(current_state) {
    case Start: std::cout << "START" << std::endl; break;
    case Increment: std::cout << "INCREMENT" << std::endl; break;
    case Decrement: std::cout << "DECREMENT" << std::endl; break;
  }

  // use averaged cte for calculation
  double avg_error = total_cte / current_steps;

  // penalize of thrown off track
  if(was_off_track == true)
    avg_error += 1000.;

  total_cte = 0.;
  current_steps = 0;
  was_off_track = false;

  // sum the d-params to check end condition
  double sum_d = 0.;

  for(int i = 0; i < PID::K_NoOf; ++i)
    sum_d += dp[i];

  // check for end condition
  if(sum_d > tolerance) {

    switch(current_state) {
      case Start:
        // init error
        best_error = avg_error;

        // modify param
        p[current_tune_parameter] += dp[current_tune_parameter];

        // change to incrementing state
        current_state = Increment;
      break;

      case Increment:
        // check if current error is better than best error
        if(avg_error < best_error) {
          // remember error
          best_error = avg_error;

          // incrementing direction is good, go further next time
          dp[current_tune_parameter] *= 1.1;

          // change to the next param as only one parameter is modified at a time
          //current_tune_parameter = (current_tune_parameter + 1) % PID::K_NoOf;

          // modify new param
          p[current_tune_parameter] += dp[current_tune_parameter];

        } else {
          // modification did not improve, so go the other way
          p[current_tune_parameter] -= 2 * dp[current_tune_parameter];

          // change state to decrementing
          current_state = Decrement;
        }
      break;

      case Decrement:
        // check if current error is better than best error
        if(avg_error < best_error) {
          // remember error
          best_error = avg_error;

          // incrementing direction is good, go further next time
          dp[current_tune_parameter] *= 1.1;

        } else {
          // reset current parameter as we did not improve
          p[current_tune_parameter] += dp[current_tune_parameter];

          // manipulate parameter change to a smaller window
          dp[current_tune_parameter] *= 0.9;

        }

        // change back to incrementing
        current_state = Increment;

        // change to the next param as only one parameter is modified at a time
        //current_tune_parameter = (current_tune_parameter + 1) % PID::K_NoOf;

        // modify new param
        p[current_tune_parameter] += dp[current_tune_parameter];

      break;
    }

    std::cout << "*** Twiddle current params: " << std::endl;
    std::cout << "p: ";
    for(int i = 0; i < PID::K_NoOf; ++i)
      std::cout << p[i] << " ";
    std::cout << std::endl << "dp: ";
    for(int i = 0; i < PID::K_NoOf; ++i)
      std::cout << dp[i] << " ";
    std::cout << std::endl;


    controller->Init(p[PID::K_P], p[PID::K_I], p[PID::K_D]);
    return false;

  } else {
    return true;
  }
}
