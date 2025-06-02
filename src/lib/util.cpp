#include "lib/util.h"
#include <cmath>

float slew(float current, float target, float rate) {
  if (target - current > rate) {
    if (current < target) {
      current += rate;
    } else if (current > target) {
      current -= rate;
    }
    return current;
  } else {
    return target;
  }
}

float avg(std::vector<float> values) {
  float sum = 0;
  for (int i = 0; i < values.size(); i++) {
    sum += values[i];
  }
  return sum / values.size();
}

float constrain180(float input) {  
  while (input > 180) {
    input -= 360;
  } while (input < -180) {
    input += 360;
  }
    return input;
}

float constrain90(float input) {
  while (input > 90) {
    input -= 180;
  } while (input < -90) {
    input += 180;
  }
  return input;
}

float degreesToRadians(float degrees) {
  return degrees * M_PI / 180;
}

float radiansToDegrees(float radians) {
  return radians * 180 / M_PI;
}

float relative90(float input, float anchor) {
  while (input > anchor + 90) {
    input -= 180;
  } while (input < anchor - 90) {
    input += 180;
  }
  return input;
}

float angleError(float current, float target) { // why tf is this a thing

  return target - current;
}
