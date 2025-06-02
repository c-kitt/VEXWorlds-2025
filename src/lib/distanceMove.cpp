// distance.cpp
#include "lib/distanceMove.hpp"
#include "robotconfig.h"  // Ensure this includes your motor and sensor declarations
#include "pros/apix.h"    // For printf function

void moveRobotDistance(double targetDistance, pros::Distance &sensor, int sensorDirection, int timeoutMs) {
    // PID constants
    double kp = 0.1;    // Reduced Proportional gain
    double ki = 0.0;    // Integral gain
    double kd = 0.7;    // Derivative gain added

    double error = 0.0;
    double prevError = 0.0;
    double derivative = 0.0;
    double output = 0.0;

    // Start the timeout timer
    int startTime = pros::millis();

    printf("Starting moveRobotDistance. Target distance: %.2f mm, Timeout: %d ms\n", targetDistance, timeoutMs);

    while (true) {
        // Check elapsed time for timeout
        int elapsedTime = pros::millis() - startTime;
        if (elapsedTime > timeoutMs) {
            printf("Timeout reached. Stopping movement.\n");
            break;
        }

        // Read the current distance from the sensor (in millimeters)
        double currentDistance = sensor.get();

        // Calculate error with sensor direction
        error = sensorDirection * (targetDistance - currentDistance);

        // Calculate derivative
        derivative = error - prevError;

        // Compute PID output
        output = (kp * error) + (kd * derivative);

        // Adjust maxVoltage based on proximity to target
        double maxVoltage = 12000; // Maximum voltage for VEX V5 motors
        if (fabs(error) < 50.0) { // Within 50 mm of target
            maxVoltage = 6000; // Reduce max voltage to half
        }

        // Limit the output to maximum motor voltage
        if (output > maxVoltage) output = maxVoltage;
        if (output < -maxVoltage) output = -maxVoltage;

        // Apply a minimum voltage threshold to overcome motor deadband
        double minVoltage = 2000; // Adjust as needed
        if (output > 0 && output < minVoltage) output = minVoltage;
        if (output < 0 && output > -minVoltage) output = -minVoltage;

        // Apply the PID output to the motors
        leftMotors.move_voltage(output);
        rightMotors.move_voltage(output);

        // Update previous error
        prevError = error;

        // Output debug information
        printf("Current Distance: %.2f mm, Error: %.2f mm, Output: %.2f mV, Elapsed Time: %d ms\n",
               currentDistance, error, output, elapsedTime);

        // Check if error is within acceptable range
        if (fabs(error) < 5.0) { // 5 mm tolerance
            printf("Target distance reached. Stopping movement.\n");
            break;
        }

        pros::delay(10); // Reduced delay for faster control loop
    }

    // Stop the motors after exiting the loop
    leftMotors.move_voltage(0);
    rightMotors.move_voltage(0);

    printf("Movement complete. Motors stopped.\n");
}
