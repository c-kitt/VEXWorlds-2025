// distance.hpp
#pragma once

#include "main.h"
#include "pros/distance.hpp"
#include "lib/liftpid.h"
#include "lib/util.h"

/**
 * @brief Moves the robot to a specified distance with a timeout.
 * 
 * @param targetDistance The target distance in millimeters.
 * @param sensor The distance sensor used for measurement.
 * @param sensorDirection The direction the sensor faces (1 or -1).
 * @param timeoutMs The maximum time in milliseconds before the function times out.
 */
void moveRobotDistance(double targetDistance, pros::Distance &sensor, int sensorDirection, int timeoutMs);
