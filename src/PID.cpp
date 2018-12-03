#include "PID.h"
#include <iostream>
#include <math.h>
using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd, bool i_reset) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
  p_error = 0;

  dp[0] = .05;
  dp[1] = .001;
  dp[2] = .2;

  bestError = 999;

  TwiddleParam = 0;
  Tstate = update;
  this->i_reset = i_reset;

}

void PID::UpdateError(double cte) {
  d_error = (cte - p_error);
  if((((cte < 0) && (p_error < 0)) || ((cte > 0) && (p_error > 0)) || !i_reset))
  {
    i_error += cte;
  }
  else
  {
    i_error = 0;
  }
  p_error = cte;
}

void PID::Reset()
{
  d_error = 0;
  p_error = 0;
  i_error = 0;
}

double PID::TotalError(double speed) {
  double p = (p_error*Kp)/(pow(2,sqrt(speed)));

  return -(p + i_error*Ki + d_error*Kd);
}
double PID::TotalError() {
  return -(p_error*Kp + i_error*Ki + d_error*Kd);
}

bool PID::UpdateAndTwiddle(double cte, double tol) {
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
      // Only is update on first past
      bestError = cte;
      p[TwiddleParam] += dp[TwiddleParam];
      Tstate = check1;
      std::cout << "UPDATE: Kp = " << p[0] << "Ki = " << p[1] << "Kd = " << p[2] << endl;
    }
    else if((Tstate == check1 || Tstate == check2) && (cte < bestError))
    {
      cout << "best error increase twiddle parameters" << cte << endl;
      bestError = cte;
      bestp[0] = Kp;
      bestp[1] = Ki;
      bestp[2] = Kd;
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
      std::cout << "UPDATE: Kp = " << p[0] << "Ki = " << p[1] << "Kd = " << p[2] << endl;
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
    Kp = p[0];
    Ki = p[1];
    Kd = p[2];

    d_error = (cte - p_error);
    p_error = cte;
    i_error += cte;
    return false;
  }
  else
  {
    std::cout << "Tolerance Reached: Kp = " << bestp[0] << "Ki = " << bestp[1] << "Kd = " << bestp[2] << endl;
    Kp = bestp[0];
    Ki = bestp[1];
    Kd = bestp[2];

    d_error = (cte - p_error);
    p_error = cte;
    i_error += cte;
    return  true;

  }
}

double PID::getBestError()
{
  return bestError;
}


