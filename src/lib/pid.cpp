/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "main.h"
#include "util.h"
#include "liftpid.h"

void allPID::variables_reset() {
  output = 0;
  target = 0;
  error = 0;
  prev_error = 0;
  integral = 0;
  time = 0;
  prev_time = 0;
}

allPID::allPID() {
  variables_reset();
  constants_set(0, 0, 0, 0);
}

allPID::Constants allPID::constants_get() { return constants; }

// PID constructor with constants
allPID::allPID(double p, double i, double d, double start_i, std::string name) {
  variables_reset();
  constants_set(p, i, d, start_i);
  name_set(name);
}

// Set PID constants
void allPID::constants_set(double p, double i, double d, double p_start_i) {
  constants.kp = p;
  constants.ki = i;
  constants.kd = d;
  constants.start_i = p_start_i;
}

// Set exit condition timeouts
void allPID::exit_condition_set(int p_small_exit_time, double p_small_error, int p_big_exit_time, double p_big_error, int p_velocity_exit_time, int p_mA_timeout) {
  exit.small_exit_time = p_small_exit_time;
  exit.small_error = p_small_error;
  exit.big_exit_time = p_big_exit_time;
  exit.big_error = p_big_error;
  exit.velocity_exit_time = p_velocity_exit_time;
  exit.mA_timeout = p_mA_timeout;
}

void allPID::target_set(double input) { target = input; }
double allPID::target_get() { return target; }

void allPID::i_reset_toggle(bool toggle) { reset_i_sgn = toggle; }
bool allPID::i_reset_get() { return reset_i_sgn; };

double allPID::compute(double current) {
  error = target - current;
  return compute_error(error, current);
}

double allPID::compute_error(double err, double current) {
  error = err;
  cur = current;

  return raw_compute();
}

double allPID::raw_compute() {
  // calculate derivative on measurement instead of error to avoid "derivative kick"
  // https://www.isa.org/intech-home/2023/june-2023/features/fundamentals-pid-control
  derivative = cur - prev_current;

  if (constants.ki != 0) {
    // Only compute i when within a threshold of target
    if (fabs(error) < constants.start_i)
      integral += error;

    // Reset i when the sign of error flips
    if (sgn(error) != sgn(prev_current) && reset_i_sgn)
      integral = 0;
  }

  output = (error * constants.kp) + (integral * constants.ki) - (derivative * constants.kd);

  prev_current = cur;
  prev_error = error;

  return output;
}

void allPID::timers_reset() {
  i = 0;
  k = 0;
  j = 0;
  l = 0;
  m = 0;
  is_mA = false;
}

void allPID::name_set(std::string p_name) {
  name = p_name;
  name_active = name == "" ? false : true;
}

void allPID::velocity_sensor_secondary_toggle_set(bool toggle) { use_second_sensor = toggle; }
bool allPID::velocity_sensor_secondary_toggle_get() { return use_second_sensor; }

void allPID::velocity_sensor_secondary_set(double secondary_sensor) { second_sensor = secondary_sensor; }
double allPID::velocity_sensor_secondary_get() { return second_sensor; }

void allPID::velocity_sensor_main_exit_set(double zero) { velocity_zero_main = zero; }
double allPID::velocity_sensor_main_exit_get() { return velocity_zero_main; }

void allPID::velocity_sensor_secondary_exit_set(double zero) { velocity_zero_secondary = zero; }
double allPID::velocity_sensor_secondary_exit_get() { return velocity_zero_secondary; }

exit_output allPID::exit_condition(bool print) {
  // If this function is called while all exit constants are 0, print an error
  if (exit.small_error == 0 && exit.small_exit_time == 0 && exit.big_error == 0 && exit.big_exit_time == 0 && exit.velocity_exit_time == 0 && exit.mA_timeout == 0) {
    
    return ERROR_NO_CONSTANTS;
  }

  // If the robot gets within the target, make sure it's there for small_timeout amount of time
  if (exit.small_error != 0) {
    if (fabs(error) < exit.small_error) {
      j += 10;
      i = 0;  // While this is running, don't run big thresh
      if (j > exit.small_exit_time) {
        timers_reset();
        
        return SMALL_EXIT;
      }
    } else {
      j = 0;
    }
  }

  // If the robot is close to the target, start a timer.  If the robot doesn't get closer within
  // a certain amount of time, exit and continue.  This does not run while small_timeout is running
  else if (exit.big_error != 0 && exit.big_exit_time != 0) {  // Check if this condition is enabled
    if (fabs(error) < exit.big_error) {
      i += 10;
      if (i > exit.big_exit_time) {
        timers_reset();
        
        return BIG_EXIT;
      }
    } else {
      i = 0;
    }
  }

  // If the motor velocity is 0, the code will timeout and set interfered to true.
  if (exit.velocity_exit_time != 0) {  // Check if this condition is enabled
    if (fabs(derivative) <= velocity_zero_main) {
      k += 10;
      if (k > exit.velocity_exit_time) {
        timers_reset();
        
        return VELOCITY_EXIT;
      }
    } else {
      k = 0;
    }
  }

  if (!use_second_sensor)
    return RUNNING;

  // If the secondary sensors velocity is 0, the code will timeout and set interfered to true.
  if (exit.velocity_exit_time != 0) {  // Check if this condition is enabled
    if (fabs(second_sensor) <= velocity_zero_secondary) {
      m += 10;
      if (m > exit.velocity_exit_time) {
        timers_reset();
        
        return VELOCITY_EXIT;
      }
    } else {
      m = 0;
    }
  }

  // printf("j: %i   i: %i   k: %i   m: %i\n", j, i, k, m);

  return RUNNING;
}

exit_output allPID::exit_condition(pros::Motor sensor, bool print) {
  // If the motors are pulling too many mA, the code will timeout and set interfered to true.
  if (exit.mA_timeout != 0) {  // Check if this condition is enabled
    if (sensor.is_over_current()) {
      l += 10;
      if (l > exit.mA_timeout) {
        timers_reset();
        
        return mA_EXIT;
      }
    } else {
      l = 0;
    }
  }

  return exit_condition(print);
}

exit_output allPID::exit_condition(std::vector<pros::Motor> sensor, bool print) {
  // If the motors are pulling too many mA, the code will timeout and set interfered to true.
  if (exit.mA_timeout != 0) {  // Check if this condition is enabled
    for (auto i : sensor) {
      // Check if 1 motor is pulling too many mA
      if (i.is_over_current()) {
        is_mA = true;
        break;
      }
      // If all of the motors aren't drawing too many mA, keep bool false
      else {
        is_mA = false;
      }
    }
    if (is_mA) {
      l += 10;
      if (l > exit.mA_timeout) {
        timers_reset();
        
        return mA_EXIT;
      }
    } else {
      l = 0;
    }
  }

  return exit_condition(print);
}