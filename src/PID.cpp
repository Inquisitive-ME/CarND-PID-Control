#include "PID.h"
#include <iostream>
using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
  p_error = 0;

  dp[0] = .2;
  dp[1] = .00005;
  dp[2] = .2;

  TwiddleParam = 0;
  Tstate = update;
  bestError = 9999;

}

void PID::UpdateError(double cte) {

  d_error = (cte - p_error);
  p_error = cte;
  i_error += cte;
}

double PID::TotalError() {
  return -(p_error*Kp + i_error*Ki + d_error*Kd);
}

void PID::UpdateAndTwiddle(double cte, double tol) {
  std::cout << "cte = " << cte << std::endl;
  double p[3] {Kp,Ki,Kd};

  double sum = 0;
  for (int i = 0; i<3; i++)
  {
    sum += dp[i];
  }
  if(sum > tol)
  {
    if(Tstate == update)
    {
      p[TwiddleParam] += dp[TwiddleParam];
      Tstate = check1;
      std::cout << "UPDATE: Kp = " << p[0] << "Ki = " << p[1] << "Kd = " << p[2] << endl;
    }
    else if((Tstate == check1 || Tstate == check2) && (cte < bestError))
    {
      cout << "best error increase twiddle parameters" << cte << endl;
      bestError = cte;
      dp[TwiddleParam] *= 1.1;
      if(TwiddleParam==2)
      {
        TwiddleParam = 0;
      }
      else
      {
        TwiddleParam++;
      }
      //Update
      p[TwiddleParam] += dp[TwiddleParam];
      Tstate = check1;
      std::cout << "UPDATE: Kp = " << p[0] << "Ki = " << p[1] << "Kd = " << p[2] << endl;
    }
    else if(Tstate == check1)
    {
      std::cout << "Error worse than " << bestError << " parameter -2x" << std::endl;
      p[TwiddleParam] -= 2*dp[TwiddleParam];
      Tstate = check2;
    }
    else if(Tstate == check2)
    {
      std::cout << "No Improvement ove " << bestError << " decrease twiddle" << std::endl;
      // put gain value back to original value
      p[TwiddleParam] += dp[TwiddleParam];
      dp[TwiddleParam] *= 0.9;
      if(TwiddleParam==2)
      {
        TwiddleParam = 0;
      }
      else
      {
        TwiddleParam++;
      }
      //Update next twiddle parameter
      p[TwiddleParam] += dp[TwiddleParam];
      Tstate = check1;
      std::cout << "UPDATE: Kp = " << p[0] << "Ki = " << p[1] << "Kd = " << p[2] << endl;
    }
  }

  Kp = p[0];
  Ki = p[1];
  Kd = p[2];

  d_error = (cte - p_error);
  p_error = cte;
  i_error += cte;
}

void PID::setBestError(double best)
{
  bestError = best;
}

