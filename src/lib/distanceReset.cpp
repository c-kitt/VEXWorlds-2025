#include "distanceReset.hpp" // Include our definitions file
#include "robotconfig.h" // Include chassis, LemLib, and distance sensors
#include <iostream>
#include <vector>

// Conversion factor from millimeters to inches, as the dist ance sensor reads in MM
constexpr double MM_TO_INCH = 1.0 / 25.4;

// Sensor port definitions (adjust based on your configuration)
#define FRONT_SENSOR_PORT 8
#define BACK_SENSOR_PORT 12
#define LEFT_SENSOR_PORT 7
#define RIGHT_SENSOR_PORT 13

double getAveragedSensorReading(pros::Distance& sensor) { 
    // Averages the first 10 sensor readings
    std::vector<double> validReadings;
    validReadings.reserve(NUM_SAMPLES);

    std::cout << "[getAveragedSensorReading] Starting readings for sensor on port " 
              << sensor.get_port() << "\n";

    for (int i = 0; i < NUM_SAMPLES; ++i) { // NUM_SAMPLES is 10 by definition
        double reading = sensor.get(); // Reads the distance sensor
        std::cout << "[getAveragedSensorReading] Reading " << i << ": " << reading << " mm\n";

        // Check for valid sensor reading
        if (reading != PROS_ERR_F && reading > 0 && reading <= MAX_VALID_DISTANCE_MM) { 
            validReadings.push_back(reading);
        } else {
            std::cout << "[getAveragedSensorReading] Invalid reading encountered: " << reading << "\n";
        }

        // Small delay between readings to allow sensor to stabilize
        pros::delay(10); 
    }

    if (validReadings.empty()) { 
        std::cout << "[getAveragedSensorReading] No valid readings collected.\n";
        return PROS_ERR_F;
    }

    // Calculate average of valid readings
    double sum = 0.0;
    for (const auto& val : validReadings) {
        sum += val;
    }
    double average = sum / validReadings.size();

    std::cout << "[getAveragedSensorReading] Valid readings count: " << validReadings.size() 
              << ", Average: " << average << " mm\n";

    return average;
}

void resetRobotPos(pros::Distance& sensor, const std::string& wallAxisDirection) {
    double distance_mm = getAveragedSensorReading(sensor); // Get average distance
    if (distance_mm == PROS_ERR_F) {   // Check for valid averaged sensor reading
        std::cout << "[resetRobotPos] Invalid averaged reading. Aborting position reset.\n";
        return;
    }

    double distance_in = distance_mm * MM_TO_INCH; // Convert mm to inches
    std::cout << "[resetRobotPos] Distance in inches (raw): " << distance_in << "\n";

    double wallPosition = 0.0;
    double sensorOffset = 0.0;
    std::string axis;

    // Determine wall position and axis
    if (wallAxisDirection == "positive_x") {
        wallPosition = WALL_X_MAX; // +71 inches
        axis = "x";
    } else if (wallAxisDirection == "negative_x") {
        wallPosition = WALL_X_MIN; // -71 inches
        axis = "x";
    } else if (wallAxisDirection == "positive_y") {
        wallPosition = WALL_Y_MAX; // +71 inches
        axis = "y";
    } else if (wallAxisDirection == "negative_y") {
        wallPosition = WALL_Y_MIN; // -71 inches
        axis = "y";
    } else {
        std::cout << "[resetRobotPos] Invalid wallAxisDirection: " << wallAxisDirection << "\n";
        return;
    }

    // Determine the sensor offset based on the sensor used
    if (sensor.get_port() == FRONT_SENSOR_PORT) {
        sensorOffset = FRONT_SENSOR_OFFSET;
    } else if (sensor.get_port() == BACK_SENSOR_PORT) {
        sensorOffset = BACK_SENSOR_OFFSET;
    } else if (sensor.get_port() == LEFT_SENSOR_PORT) {
        sensorOffset = LEFT_SENSOR_OFFSET;
    } else if (sensor.get_port() == RIGHT_SENSOR_PORT) {
        sensorOffset = RIGHT_SENSOR_OFFSET;
    } else {
        std::cout << "[resetRobotPos] Unknown sensor port: " << sensor.get_port() << "\n";
        return;
    }

    std::cout << "[resetRobotPos] Wall position: " << wallPosition 
              << ", Sensor offset: " << sensorOffset << ", Axis: " << axis << "\n";

    // Calculate the robot's position along the axis
    double robotPosition = wallPosition + distance_in + sensorOffset;

    // For positive walls (wallPosition > 0), we need to subtract distances
    if (wallPosition > 0) {
        robotPosition = wallPosition - distance_in - sensorOffset;
    }

    std::cout << "[resetRobotPos] Calculated robot position along " << axis << ": " << robotPosition << "\n";

    // Get the current pose
    lemlib::Pose currentPose = chassis.getPose();
    std::cout << "[resetRobotPos] Current Pose: x=" << currentPose.x 
              << ", y=" << currentPose.y 
              << ", theta=" << currentPose.theta << "\n";

    // Update the robot's position along the specified axis
    if (axis == "x") {
        chassis.setPose(robotPosition, currentPose.y, currentPose.theta);
    } else if (axis == "y") {
        chassis.setPose(currentPose.x, robotPosition, currentPose.theta);
    }

    lemlib::Pose newPose = chassis.getPose();
    std::cout << "[resetRobotPos] Updated Pose: x=" << newPose.x 
              << ", y=" << newPose.y 
              << ", theta=" << newPose.theta << "\n";
}
