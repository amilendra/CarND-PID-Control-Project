#include "PID.h"
#include <cmath>
#include <iostream>
#include <limits>
#include <numeric>
#include <vector>

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
  p_error = 0.0;
  d_error = 0.0;
  i_error = 0.0;
}

void PID::UpdateError(double cte) {
  d_error = cte - p_error;
  p_error = cte;
  i_error += cte;
#ifdef TWIDDLE
  static int idx = 0;
  static int step = 0;
  static const int IGNORE_STEPS = 0;
  static const int EVALUATE_STEPS = 50;
  static double err = 0;
  static double best_err = std::numeric_limits<double>::max();
  static std::vector<double> p{Kp, Ki, Kd};
  static std::vector<double> dp{Kp / 2, Ki / 2, Kd / 2};
  static bool first_round = true;

  if (std::accumulate(dp.begin(), dp.end(), 0.0) < 0.002) return;

  // ignore the first IGNORE_STEPS
  if (step < IGNORE_STEPS) {
    step++;
    return;
  }
  // sum error until EVALUATE_STEPS
  else if (step < EVALUATE_STEPS) {
    step++;
    err += pow(cte, 2);
    return;
  } else if (step >= EVALUATE_STEPS) {
    step = 0;
    // Since we set best_err to the max double value, err will always be less
    // tha best_err. In that case no point in tuning parameters, so, for the
    // first round, just update best_err and move to the next run.
    if (first_round) {
      first_round = false;
      return;
    }
  }

  // Choose parameters
  enum PHASE { ADD_ONE, MINUS_ONE, BACK_TO_ZERO, CHANGE_CYCLE };
  static int phase = ADD_ONE;
  if (err < best_err) {
    best_err = err;
    dp[idx] *= 1.1;
    cout << "better params found: best err: " << best_err
         << " (P, I, D) : " << Kp << ", " << Ki << ", " << Kd << endl;
    // Move to the next parameter
    idx = (idx + 1) % 3;
    // and start the cycle.
    phase = ADD_ONE;
  }
  if (phase == ADD_ONE) {
    p[idx] += dp[idx];
    phase++;
  } else if (phase == MINUS_ONE) {
    p[idx] -= 2 * dp[idx];
    phase++;
  } else if (phase == BACK_TO_ZERO) {
    p[idx] += dp[idx];
    dp[idx] *= 0.9;
    phase++;
  } else {
    phase = ADD_ONE;
    idx = (idx + 1) % 3;
  }

  switch (idx) {
    case 0:
      Kp = p[idx];
      break;
    case 1:
      Ki = p[idx];
      break;
    case 2:
      Kd = p[idx];
      break;
    default:
      std::cout << "Error: Cant be here" << std::endl;
  }

  if (phase == ADD_ONE) {
    idx = (idx + 1) % 3;
  }
  err = 0;
  cout << "params updated: "
       << " (P, I, D) : " << Kp << ", " << Ki << ", " << Kd << endl;
#endif
}

double PID::TotalError() {
  return -Kp * p_error + -Kd * d_error + -Ki * i_error;
}
