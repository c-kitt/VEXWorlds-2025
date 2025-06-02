#include "cornerReset.hpp"
#include "robotconfig.h"
#include "lemlib/chassis/chassis.hpp"
#include <cmath>
#include <iostream>

// If you're in a PROS environment, you can do #include "pros/apix.h" or "main.h"
// then replace 'delayMs(10)' with 'pros::delay(10)'.
static void delayMs(int ms) {
#ifdef __PROS__
    pros::delay(ms);
#else
    // Minimal placeholder (no real delay on a normal C++ environment)
#endif
}

// -----------------------------------------------------------------------------
// 1) Field & Sensor Offsets
// -----------------------------------------------------------------------------

static constexpr double FIELD_WALL = 71.0; // +/-71 inches from center of field

// If your local X-axis is left->right (+ to the right), and local Y-axis is front->back (+ forward),
// then a "left" sensor might be X=-5, Y=0, and a "right" sensor might be X=+5, Y=0.
static constexpr double LEFT_SENSOR_OFFSET_X  = -5.0;
static constexpr double LEFT_SENSOR_OFFSET_Y  =  +3.0;
static constexpr double RIGHT_SENSOR_OFFSET_X = +5.0;
static constexpr double RIGHT_SENSOR_OFFSET_Y =  +3.0;

// -----------------------------------------------------------------------------
// 2) Helper Functions to Read & Average Sensors
// -----------------------------------------------------------------------------

// A single read from the left distance sensor (returns mm). 
// If you have a pros::Distance sensor, you might do: leftSensor.get() 
static double getLeftSensorRawMM() {
    double mm = leftSensor.get();  // placeholder: read your actual sensor
    return mm;
}

// A single read from the right distance sensor (returns mm).
static double getRightSensorRawMM() {
    double mm = rightSensor.get(); // placeholder
    return mm;
}

// We'll take 10 readings from each sensor, printing them out, and average:
static double getLeftDistanceInchesAveraged() {
    const int NUM_SAMPLES = 10;
    double sum = 0.0;

    std::cout << "[cornerReset] Collecting " << NUM_SAMPLES 
              << " samples from LEFT sensor...\n";

    for (int i = 0; i < NUM_SAMPLES; ++i) {
        double rawMM = getLeftSensorRawMM();
        std::cout << "  [cornerReset] Left Reading #" << i << ": " 
                  << rawMM << " mm\n";

        sum += rawMM;
        delayMs(10); // short delay between samples
    }

    double avgMM = sum / NUM_SAMPLES;
    double avgInches = avgMM / 25.4; // convert mm to inches

    std::cout << "[cornerReset] LEFT sensor average (mm) = " << avgMM 
              << " => " << avgInches << " in\n";

    return avgInches;
}

static double getRightDistanceInchesAveraged() {
    const int NUM_SAMPLES = 10;
    double sum = 0.0;

    std::cout << "[cornerReset] Collecting " << NUM_SAMPLES 
              << " samples from RIGHT sensor...\n";

    for (int i = 0; i < NUM_SAMPLES; ++i) {
        double rawMM = getRightSensorRawMM();
        std::cout << "  [cornerReset] Right Reading #" << i << ": " 
                  << rawMM << " mm\n";

        sum += rawMM;
        delayMs(10);
    }

    double avgMM = sum / NUM_SAMPLES;
    double avgInches = avgMM / 25.4;

    std::cout << "[cornerReset] RIGHT sensor average (mm) = " << avgMM 
              << " => " << avgInches << " in\n";

    return avgInches;
}

// -----------------------------------------------------------------------------
// 3) cornerReset function
// -----------------------------------------------------------------------------
//
// Quadrant logic (1..4), in your coordinate system:
//   - Quadrant 1 => top-right => (x=+71, y=+71)
//       left -> X => sensorLeftX  =  71 - distLeft
//       right -> Y => sensorRightY = 71 - distRight
//   - Quadrant 2 => bottom-right => (x=+71, y=-71)
//       left -> Y => sensorLeftY  = -71 + distLeft
//       right -> X => sensorRightX = 71  - distRight
//   - Quadrant 3 => bottom-left => (x=-71, y=-71)
//       left -> X => sensorLeftX  = -71 + distLeft
//       right -> Y => sensorRightY = -71 + distRight
//   - Quadrant 4 => top-left => (x=-71, y=+71)
//       right -> X => sensorRightX = -71 + distRight
//       left -> Y  = 71 - distLeft
//
// 0° heading = facing +Y, angles increase clockwise.
//
// The local->global transform for (xLocal, yLocal) is:
//   xGlobal = xLocal * sin(θ) + yLocal * cos(θ)
//   yGlobal = -xLocal * cos(θ) + yLocal * sin(θ)
//
void cornerReset(int quadrant) {
    std::cout << "\n[cornerReset] Starting corner reset with quadrant=" 
              << quadrant << "...\n";

    // 1) Read the current heading from the IMU
    double headingDeg = chassis.getPose().theta; // or your IMU call
    double thetaRad   = headingDeg * M_PI / 180.0;

    std::cout << "[cornerReset] Robot heading: " << headingDeg 
              << " deg (0°=+Y, CW)\n";

    // 2) Read & average the left & right sensor distances
    double distLeft  = getLeftDistanceInchesAveraged();  
    double distRight = getRightDistanceInchesAveraged();

    // 3) Compute each sensor's global offset from the robot center
    double leftGlobX  = LEFT_SENSOR_OFFSET_X * std::sin(thetaRad)
                      + LEFT_SENSOR_OFFSET_Y * std::cos(thetaRad);
    double leftGlobY  = -LEFT_SENSOR_OFFSET_X * std::cos(thetaRad)
                      + LEFT_SENSOR_OFFSET_Y * std::sin(thetaRad);

    double rightGlobX = RIGHT_SENSOR_OFFSET_X * std::sin(thetaRad)
                      + RIGHT_SENSOR_OFFSET_Y * std::cos(thetaRad);
    double rightGlobY = -RIGHT_SENSOR_OFFSET_X * std::cos(thetaRad)
                      + RIGHT_SENSOR_OFFSET_Y * std::sin(thetaRad);

    std::cout << "[cornerReset] Left sensor global offset = ("
              << leftGlobX << ", " << leftGlobY << ")\n"
              << "[cornerReset] Right sensor global offset = ("
              << rightGlobX << ", " << rightGlobY << ")\n";

    // 4) Quadrant-based assignment of sensorLeftX, sensorLeftY, sensorRightX, sensorRightY
    double sensorLeftX  = 0.0, sensorLeftY  = 0.0;
    double sensorRightX = 0.0, sensorRightY = 0.0;

    switch (quadrant) {
    case 1:
        // top-right
        sensorLeftX  =  FIELD_WALL - distLeft;   // left->X
        sensorRightY =  FIELD_WALL - distRight;  // right->Y
        break;

    case 2:
        // bottom-right
        sensorLeftY  = -FIELD_WALL + distLeft;   // left->Y
        sensorRightX =  FIELD_WALL - distRight;  // right->X
        break;

    case 3:
        // bottom-left
        sensorLeftX  = -FIELD_WALL + distLeft;   // left->X
        sensorRightY = -FIELD_WALL + distRight;  // right->Y
        break;

    case 4:
        // top-left
        sensorRightX = -FIELD_WALL + distRight;  // right->X
        sensorLeftY  =  FIELD_WALL - distLeft;   // left->Y
        break;

    default:
        std::cout << "[cornerReset] Invalid quadrant=" << quadrant << "\n";
        return;
    }

    std::cout << "[cornerReset] After quadrant logic:\n"
              << "  sensorLeftX=" << sensorLeftX << ", sensorLeftY=" << sensorLeftY << "\n"
              << "  sensorRightX=" << sensorRightX << ", sensorRightY=" << sensorRightY << "\n";

    // 5) Solve for the robot center
    double xCenter = 0.0;
    double yCenter = 0.0;

    if (sensorLeftX != 0.0) {
        xCenter = sensorLeftX - leftGlobX;
    }
    if (sensorLeftY != 0.0) {
        yCenter = sensorLeftY - leftGlobY;
    }
    if (sensorRightX != 0.0) {
        xCenter = sensorRightX - rightGlobX;
    }
    if (sensorRightY != 0.0) {
        yCenter = sensorRightY - rightGlobY;
    }

    std::cout << "[cornerReset] Computed center (xCenter, yCenter)=("
              << xCenter << ", " << yCenter << ")\n";

    // 6) Set the final pose in your odometry
    chassis.setPose(xCenter, yCenter, headingDeg);

    std::cout << "[cornerReset] quadrant=" << quadrant
              << " => Final Pose= x=" << xCenter 
              << ", y=" << yCenter 
              << ", heading=" << headingDeg << " deg\n\n";
}
