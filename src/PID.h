#ifndef PID_H
#define PID_H

class PID {
 public:
  /**
   * Constructor
   */
  PID();

  /**
   * Destructor.
   */
  virtual ~PID();

  /**
   * Initialize PID.
   * @param (Kp_, Ki_, Kd_) The initial PID coefficients
   */
  void Init(double Kp_, double Ki_, double Kd_);

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  void UpdateError(double cte);

  /**
   * Calculate the total PID error.
   * @output The total PID error
   */
  double TotalError();

  /**
   * Calculates the control steering angle
   * @param lower_limit The lower limit of the resulting control value
   * @param upper_limit The upper limit of the resulting control value
   * @output Control value of PID controller
   */
  double Control(double lower_limit, double upper_limit);

 private:
  enum PidCoefficients {
    K_P = 0,
    K_I,
    K_D,
    K_NoOf
  };

  /**
   * PID Errors
   */
  double error[K_NoOf];

  /**
   * PID Coefficients
   */
  double K[K_NoOf];
};

#endif  // PID_H
