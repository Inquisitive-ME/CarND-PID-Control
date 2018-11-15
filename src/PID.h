#ifndef PID_H
#define PID_H

enum TwiddleState {update, check1, check2};

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  double bestError;
  double dp[3];
  int TwiddleParam;

  TwiddleState Tstate;



  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  void UpdateAndTwiddle(double cte, double tol);

  void setBestError(double best);
};

#endif /* PID_H */
