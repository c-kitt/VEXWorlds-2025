#pragma once
#include "main.h"

template <typename T> constexpr T sgn(T value) { return value < 0 ? -1 : 1; }

float slew(float current, float target, float rate);

float avg(std::vector<float> values);

float constrain180(float input);

float constrain90(float input);

float relative90(float input, float current);

float degreesToRadians(float degrees);

float radiansToDegrees(float radians);

float angleError(float current, float target);

enum exit_output { RUNNING = 1,
                   SMALL_EXIT = 2,
                   BIG_EXIT = 3,
                   VELOCITY_EXIT = 4,
                   mA_EXIT = 5,
                   ERROR_NO_CONSTANTS = 6 };