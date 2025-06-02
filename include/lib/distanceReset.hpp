#ifndef DISTANCE_RESET_HPP //Defines the file
#define DISTANCE_RESET_HPP
#include "lemlib/api.hpp"
#include "robotconfig.h"
#include <string>

// Field dimensions (in inches)
constexpr double FIELD_SIZE = 142.0; // Field spans from -72 to +72 inches
// Wall coordinates
constexpr double WALL_X_MIN = -71.0; // Negative X wall at X = -71 inches
constexpr double WALL_X_MAX = 71.0;  // Positive X wall at X = +71 inches
constexpr double WALL_Y_MIN = -71.0; // Negative Y wall at Y = -71 inches
constexpr double WALL_Y_MAX = 71.0;  // Positive Y wall at Y = +71 inches
// Sensor offsets from the robot's center along each axis (in inches)
constexpr double FRONT_SENSOR_OFFSET = 2.7; 
constexpr double BACK_SENSOR_OFFSET = -1.5;   
constexpr double LEFT_SENSOR_OFFSET = 5.2;   
constexpr double RIGHT_SENSOR_OFFSET = 5.2;  

// Filtering constants
constexpr int NUM_SAMPLES = 10;                // Averages 10 values of the distance sensor
constexpr double MAX_VALID_DISTANCE_MM = 3000.0; // Maximum valid distance in mm

void resetRobotPos(pros::Distance& sensor, const std::string& wallAxisDirection); //The reset function

double getAveragedSensorReading(pros::Distance& sensor); //The average function
#endif // DISTANCE_RESET_HPP