#ifndef PIDAUTOTUNE_H_
#define PIDAUTOTUNE_H_

#include "PID.h"

class PIDAutotune {
public:
  PIDAutotune(PID* cont);

  bool didFinishRun();
  bool cteIndicatesOffTrack(double cte, double speed);

  void addUpError(double cte);

  bool twiddle(double tolerance);

private:
  enum TwiddleState {
    Start = 0,
    Increment,
    Decrement
  };

  // ignore first steps to avoid collecting errors in the speed up phase
  static const unsigned int init_steps = 100;

  TwiddleState current_state;

  PID *controller;
  double p[PID::K_NoOf];
  double dp[PID::K_NoOf];
  unsigned int current_steps;
  double total_cte;
  double best_error;
  unsigned int current_tune_parameter;
  bool was_off_track;
  unsigned int round;
};

#endif /* PIDAUTOTUNE_H_ */
