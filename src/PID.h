#ifndef PID_H
#define PID_H

#include <uWS/uWS.h>
#include "json.hpp"
#include <vector>
#include <numeric>

using std::vector;

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
   * @brief Initialize PID coefficients (and errors, if needed)
   * 
   * @param Kp_ Control Constant -> Proportional
   * @param Ki_ Control Constant -> Integral
   * @param Kd_ Control Constant -> Differential
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
   * @brief Execute update for steering angle and throttle
   * 
   * @param steering_angle steering angle command
   * @param throttle throttle command
   * @param ws WebSocket to communicate with client
   */

  void ExecuteUpdate(
      const double steering_angle,
      const double throttle,
      uWS::WebSocket<uWS::SERVER> ws);

  /**
   * @brief Twiddle the params and return optimal params
   * 
   * @param tolerance minimum the sum of all params can be
   * @param INCREMENT increment rate for each controller parameter
   * @param MAX_STEPS attempt amount for twiddling
   * @return true if found better params
   * @return false if kept original params
   */

  bool Twiddle(const vector <double> INCREMENT, const int MAX_STEPS);

  double Kp_(){return Kp;}
  double Ki_(){return Ki;}
  double Kd_(){return Kd;}

 private:
  /**
   * PID Errors
   */
  double p_error;
  double i_error;
  double d_error;

  /**
   * PID Coefficients
   */ 
  double Kp;
  double Ki;
  double Kd;

  /**
   * Other
   */
  double prev_CTE;
  bool have_i_twiddled;
};

#endif  // PID_H