#pragma once

#include "lib/intake.hpp"
#include "lib/lift.hpp"
#include "robotconfig.h"
#include "lib/cornerReset.hpp"
#include "lib/distanceReset.hpp"
#include "lib/distanceMove.hpp"
#include <cmath>
#include <iostream>

// Use degrees per inch for a 2.75" wheel in direct drive.
#define DEGREES_PER_INCH 41.66 // Approx.

inline void pidfwd(float distanceInInches, float errorRangeInInches, uint32_t timeoutMS) {
  // Convert inches to motor degrees
  float distanceDeg   = distanceInInches   * DEGREES_PER_INCH;
  float errorRangeDeg = errorRangeInInches * DEGREES_PER_INCH;

  // Zero motor encoders
  chassis.drivetrain.leftMotors->tare_position();
  chassis.drivetrain.rightMotors->tare_position();

  // Record the starting average position in degrees
  float startingPosDeg = (chassis.drivetrain.leftMotors->get_position() +
                          chassis.drivetrain.rightMotors->get_position()) / 2.0;

  // Initial error
  float currentDistanceDeg = 0.0;
  float error = distanceDeg - currentDistanceDeg;
  chassis.lateralPID.reset();

  // Start time for timeout
  uint32_t startTime = pros::millis();

  // Move until within error tolerance OR until we hit our timeout
  while (fabs(error) > errorRangeDeg && (pros::millis() - startTime) < timeoutMS) {
    // Calculate PID output
    float output = chassis.lateralPID.update(error);

    // Drive forward
    chassis.arcade(output, 0);

    // Recalculate current position and error
    float leftPos  = chassis.drivetrain.leftMotors->get_position();
    float rightPos = chassis.drivetrain.rightMotors->get_position();
    currentDistanceDeg = ((leftPos + rightPos) / 2.0) - startingPosDeg;
    error = distanceDeg - currentDistanceDeg;

    pros::delay(10);
  }

  // Optionally stop motors at the end
  chassis.arcade(0, 0);
}

inline void testAuto(){
  chassis.setPose(-48,24,180);
}


inline void moveUntilX(float desiredX, float desiredY, float targetX, int timeout) {
    // Initiate backward movement to the desired point
    std::cout << "Starting moveToPoint to (" << desiredX << ", " << desiredY << ") with timeout " << timeout << "ms\n";
    chassis.moveToPoint(desiredX, desiredY, timeout, {.forwards = false, .maxSpeed = 100});

    // Continuously monitor the robot's x-position
    while (chassis.isInMotion()) {
        float currentX = chassis.getPose().x;
        std::cout << "Current X: " << currentX << ", Target X: " << targetX << "\n";

        if (currentX <= targetX) {
            std::cout << "Target X reached. Canceling motion.\n";
            chassis.cancelMotion();
            doinker.set_value(false);
            break;
        }
        pros::delay(10); // Short delay to prevent CPU overutilization
    }
    std::cout << "Motion completed or canceled.\n";
}

inline void driveBackHoldAngle(float power, float targetHeading, float stopX, int timeout) {
    int start = pros::millis();

    while (pros::millis() - start < timeout) {
        float currentX = chassis.getPose().x;
        float currentHeading = chassis.getPose().theta;

        // Normalize heading error to [-180, 180]
        float headingError = targetHeading - currentHeading;
        if (headingError > 180) headingError -= 360;
        if (headingError < -180) headingError += 360;

        // Heading P control
        float kP = 5;
        float correction = kP * headingError;

        float leftPower = -power + correction;
        float rightPower = -power - correction;

        int clippedLeft = std::max(-127, std::min(127, (int)leftPower));
        int clippedRight = std::max(-127, std::min(127, (int)rightPower));

        leftMotors.move(clippedLeft);
        rightMotors.move(clippedRight);

        // Stop once passed stopX moving backwards
        if ((stopX - currentX) * power < 0) break;

        pros::delay(10);
    }

    leftMotors.move(0);
    rightMotors.move(0);
    pros::delay(200);
    doinker.set_value(false);
    rightArm.set_value(false);
    pros::delay(200);
}

inline void driveBackHoldAngleRed(float power, float targetHeading, float stopX, int timeout) {
    int start = pros::millis();

    while (pros::millis() - start < timeout) {
        float currentX = chassis.getPose().x;
        float currentHeading = chassis.getPose().theta;

        // Normalize heading error to [-180, 180]
        float headingError = targetHeading - currentHeading;
        if (headingError > 180) headingError -= 360;
        if (headingError < -180) headingError += 360;

        // Heading P control
        float kP = 5;
        float correction = kP * headingError;

        float leftPower = -power + correction;
        float rightPower = -power - correction;

        int clippedLeft = std::max(-127, std::min(127, (int)leftPower));
        int clippedRight = std::max(-127, std::min(127, (int)rightPower));

        leftMotors.move(clippedLeft);
        rightMotors.move(clippedRight);

        // ✅ RED SIDE: moving from -18 to -30 → stop when currentX <= stopX
        if (currentX <= stopX) break;

        pros::delay(10);
    }

    leftMotors.move(0);
    rightMotors.move(0);
    pros::delay(200);
    doinker.set_value(false);
    rightArm.set_value(false);
    pros::delay(200);
}

//RED
inline void redSAWP(){
chassis.setPose(-55, 16, 180);
//Score Alliance Stake

chassis.moveToPose(-60, 12, 219, 550, {.horizontalDrift = 10, .maxSpeed = 110});
lift.setState(lib::LiftState::AllianceMid);
chassis.waitUntilDone();

lift.setState(lib::LiftState::AllianceScore);
pros::delay(150);

chassis.moveToPoint(-54, 16, 350, {.forwards = false, .maxSpeed = 110});
chassis.waitUntilDone();


//Clamp Goal

chassis.moveToPoint(-24, 24, 1150, {.forwards = false, .maxSpeed = 90});
pros::delay(500);
lift.setState(lib::LiftState::AllianceScore);
pros::delay(650);
clamp.set_value(true);
chassis.waitUntilDone();
lift.setState(lib::LiftState::Stored);
//Score 3 Rings 

//score rings
chassis.turnToPoint(-9.5,37,350, {.maxSpeed = 100});
chassis.waitUntilDone();

intake.setState(lib::IntakeState::In);

chassis.moveToPoint(-9.5, 37, 750, {.maxSpeed = 100});
chassis.waitUntilDone();

pros::delay(150);

chassis.moveToPoint(-8, 57, 750, {.maxSpeed = 70});
chassis.waitUntilDone();

//protected ring
chassis.moveToPoint(-24, 24, 750, {.forwards = false, .maxSpeed = 110});
chassis.waitUntilDone();

chassis.turnToPoint(-24, 44, 250);
chassis.waitUntilDone();

chassis.moveToPoint(-24, 44, 850, {.maxSpeed = 110});
chassis.waitUntilDone();

//Get Next Ring
chassis.moveToPoint(-24, 24, 750, {.forwards = false, .maxSpeed = 110});
chassis.waitUntilDone();

chassis.turnToPoint(-48, 0, 500, {.maxSpeed = 100});
chassis.waitUntilDone();

leftArm.set_value(true);
intake.setJamEnabled(false);

//intake.setState(lib::IntakeState::SpinUntil);
intake.setState(lib::IntakeState::In);

chassis.moveToPoint(-48, 0, 850, {.maxSpeed = 70});
//pros::delay(650);
//clamp.set_value(false);
chassis.waitUntilDone();
leftArm.set_value(false);

chassis.moveToPoint(-40, 10, 500, {.forwards = false, .maxSpeed = 110});
chassis.waitUntilDone();

chassis.moveToPoint(-60, -48, 1300, {.forwards = true, .maxSpeed = 110});
pros::delay(1050);
intake.setState(lib::IntakeState::SpinUntil);
chassis.waitUntilDone();

chassis.moveToPoint(-50, -28, 500, {.forwards = false, .maxSpeed = 110});
chassis.waitUntilDone();
clamp.set_value(false);

chassis.moveToPoint(-58, -40, 500, {.forwards = true, .maxSpeed = 110});
chassis.waitUntilDone();

//Clamp 2nd Mogo
chassis.turnToPoint(-24, -24, 250, {.forwards=false, .maxSpeed = 100});
chassis.waitUntilDone();


chassis.moveToPoint(-24, -24, 1250, {.forwards = false, .maxSpeed = 87.5});
pros::delay(1200);
clamp.set_value(true);
chassis.waitUntilDone();

//One ring
chassis.turnToPoint(-24, -48, 350);
chassis.waitUntilDone();

intake.setState(lib::IntakeState::In);

chassis.moveToPoint(-24, -46, 750, {.maxSpeed = 110});
chassis.waitUntilDone();

//bar touch

chassis.moveToPoint(-24, -24, 750, { .forwards = false, .maxSpeed = 110});
chassis.waitUntilDone();

chassis.moveToPoint(-12, -12, 1000, {.forwards = true, .maxSpeed = 100});
lift.setState(lib::LiftState::Recieve);
chassis.waitUntilDone();

}

inline void red7Ring(){
chassis.setPose(-55, 16, 180);
//Score Alliance Stake

chassis.moveToPose(-60, 12, 219, 550, {.horizontalDrift = 10, .maxSpeed = 110});
lift.setState(lib::LiftState::AllianceMid);
chassis.waitUntilDone();

lift.setState(lib::LiftState::AllianceScore);
pros::delay(150);

chassis.moveToPoint(-54, 16, 350, {.forwards = false, .maxSpeed = 110});
chassis.waitUntilDone();


//Clamp Goal

chassis.moveToPoint(-24, 24, 1150, {.forwards = false, .maxSpeed = 90});
pros::delay(500);
lift.setState(lib::LiftState::AllianceScore);
pros::delay(650);
clamp.set_value(true);
chassis.waitUntilDone();
lift.setState(lib::LiftState::Stored);
//Score 3 Rings 

//score rings
chassis.turnToPoint(-9,37,350, {.maxSpeed = 100});
chassis.waitUntilDone();

intake.setState(lib::IntakeState::In);

chassis.moveToPoint(-9, 37, 750, {.maxSpeed = 100});
chassis.waitUntilDone();

pros::delay(150);

chassis.moveToPoint(-7, 56, 750, {.maxSpeed = 70});
chassis.waitUntilDone();

//protected ring
chassis.moveToPoint(-24, 24, 750, {.forwards = false, .maxSpeed = 110});
chassis.waitUntilDone();

chassis.turnToPoint(-24, 44, 250);
chassis.waitUntilDone();

chassis.moveToPoint(-24, 44, 850, {.maxSpeed = 110});
chassis.waitUntilDone();

//Get Next Ring
chassis.moveToPoint(-24, 24, 1000, {.forwards = false, .maxSpeed = 110});
chassis.waitUntilDone();

//ring stack
chassis.turnToPoint(-48, 48, 500);
chassis.waitUntilDone();

chassis.moveToPoint(-55, 55, 1000, {.maxSpeed = 100});
chassis.waitUntilDone();

chassis.moveToPoint(-64, 64, 750, {.maxSpeed = 60});
chassis.waitUntilDone();

chassis.moveToPoint(-46, 46, 750, {.forwards = false, .maxSpeed = 110});
chassis.waitUntilDone();

chassis.moveToPoint(-60, 60, 1000, {.maxSpeed = 80});
chassis.waitUntilDone();

chassis.moveToPoint(-48, 48, 1000, {.forwards = false, .maxSpeed = 100});
chassis.waitUntilDone();

//middle ring stack alliance
chassis.turnToPoint(-48, 0, 500, {.maxSpeed = 100});
chassis.waitUntilDone();

leftArm.set_value(true);
intake.setJamEnabled(false);

chassis.moveToPoint(-48, 0, 1000, {.maxSpeed = 90});
chassis.waitUntilDone();

chassis.moveToPoint(-45, 3, 1000, {.maxSpeed = 90});
chassis.waitUntilDone();
leftArm.set_value(false);
}

inline void red6RingNegative(){
chassis.setPose(-55, 16, 180);
//Score Alliance Stake

chassis.moveToPose(-60, 12, 219, 550, {.horizontalDrift = 10, .maxSpeed = 110});
lift.setState(lib::LiftState::AllianceMid);
chassis.waitUntilDone();

lift.setState(lib::LiftState::AllianceScore);
pros::delay(150);

chassis.moveToPoint(-54, 16, 350, {.forwards = false, .maxSpeed = 110});
chassis.waitUntilDone();


//Clamp Goal

chassis.moveToPoint(-24, 24, 1150, {.forwards = false, .maxSpeed = 90});
pros::delay(500);
lift.setState(lib::LiftState::AllianceScore);
pros::delay(650);
clamp.set_value(true);
chassis.waitUntilDone();
lift.setState(lib::LiftState::Stored);
//Score 3 Rings 

//score rings
chassis.turnToPoint(-9,37,350, {.maxSpeed = 100});
chassis.waitUntilDone();

intake.setState(lib::IntakeState::In);

chassis.moveToPoint(-9, 37, 750, {.maxSpeed = 100});
chassis.waitUntilDone();

pros::delay(150);

chassis.moveToPoint(-7, 56, 750, {.maxSpeed = 70});
chassis.waitUntilDone();

//protected ring
chassis.moveToPoint(-24, 24, 750, {.forwards = false, .maxSpeed = 110});
chassis.waitUntilDone();

chassis.turnToPoint(-24, 44, 250);
chassis.waitUntilDone();

chassis.moveToPoint(-24, 44, 850, {.maxSpeed = 110});
chassis.waitUntilDone();

//Get Next Ring
chassis.moveToPoint(-24, 24, 1000, {.forwards = false, .maxSpeed = 110});
chassis.waitUntilDone();

//ring stack
chassis.turnToPoint(-48, 48, 500);
chassis.waitUntilDone();

chassis.moveToPoint(-55, 55, 1000, {.maxSpeed = 100});
chassis.waitUntilDone();

chassis.moveToPoint(-64, 64, 750, {.maxSpeed = 60});
chassis.waitUntilDone();

chassis.moveToPoint(-46, 46, 750, {.forwards = false, .maxSpeed = 110});
chassis.waitUntilDone();

chassis.moveToPoint(-60, 60, 1000, {.maxSpeed = 80});
chassis.waitUntilDone();

chassis.moveToPoint(-48, 48, 1000, {.forwards = false, .maxSpeed = 100});
chassis.waitUntilDone();

chassis.turnToPoint(0, 0, 750, {.maxSpeed = 100});
chassis.waitUntilDone();

chassis.moveToPoint(-10, 10, 1000, {.maxSpeed = 80});
lift.setState(lib::LiftState::Recieve);
chassis.waitUntilDone();

}

inline void red6RingPositive(){
chassis.setPose(-55, -16, 0);
//Score Alliance Stake

chassis.moveToPose(-60, -12, -40, 650, {.horizontalDrift = 10, .maxSpeed = 110});
//lift.setState(lib::LiftState::AllianceMid);
chassis.waitUntilDone();

//lift.setState(lib::LiftState::AllianceScore);
pros::delay(150);

chassis.moveToPoint(-54, -16, 350, {.forwards = false, .maxSpeed = 110});
chassis.waitUntilDone();

//Clamp Goal
chassis.moveToPoint(-24, -24, 1150, {.forwards = false, .maxSpeed = 90});
pros::delay(500);
//lift.setState(lib::LiftState::AllianceScore);
pros::delay(500);
clamp.set_value(true);
chassis.waitUntilDone();
//lift.setState(lib::LiftState::Stored);

chassis.turnToPoint(-24, -44, 500);
chassis.waitUntilDone();

intake.setState(lib::IntakeState::In);

chassis.moveToPoint(-24, -44, 850, {.maxSpeed = 110});
chassis.waitUntilDone();

//Get Next Ring
chassis.moveToPoint(-24, -24, 950, {.forwards = false, .maxSpeed = 110});
chassis.waitUntilDone();

//ring stack
chassis.turnToPoint(-48, -48, 500, {.maxSpeed = 100});
chassis.waitUntilDone();

chassis.moveToPoint(-48, -48, 1000, {.maxSpeed = 100});
chassis.waitUntilDone();


chassis.moveToPoint(-67, -67, 1150, {.maxSpeed = 60});
chassis.waitUntilDone();

chassis.moveToPoint(-48, -48, 750, {.forwards = false, .maxSpeed = 90});
chassis.waitUntilDone();

chassis.moveToPoint(-58,-58, 1000, {.maxSpeed = 90});
chassis.waitUntilDone();

chassis.moveToPoint(-48, -48, 1000, {.forwards = false, .maxSpeed = 100});
chassis.waitUntilDone();

//middle ring stack alliance
chassis.turnToPoint(-48, 0, 500, {.maxSpeed = 100});
chassis.waitUntilDone();

leftArm.set_value(true);
intake.setJamEnabled(false);

chassis.moveToPoint(-48, 0, 1250, {.maxSpeed = 80});
pros::delay(1000);
chassis.waitUntilDone();
leftArm.set_value(false);
pros::delay(750);

chassis.turnToPoint(-24, -24, 500, {.maxSpeed = 100});
chassis.waitUntilDone();

chassis.moveToPoint(-24, -24, 850, {.maxSpeed = 100});
chassis.waitUntilDone();

chassis.turnToPoint(0, 0, 500, {.maxSpeed = 100});
chassis.waitUntilDone();

chassis.moveToPoint(-12, -12, 1000, {.maxSpeed = 60});
lift.setState(lib::LiftState::Recieve);
chassis.waitUntilDone();
}

inline void redGoalRush(){
    chassis.setPose(-58, -48, 95);
    leftArm.set_value(false);
    chassis.lateralSettings.slew = 0;

    intake.setState(lib::IntakeState::SpinUntil);
    rightArm.set_value(true);

    chassis.moveToPoint(-18, -52, 1400, {.forwards = true, .maxSpeed = 127});

    const int maxSensorWait = 1500; // 1.5 second cutoff
    int start = pros::millis();
    bool clamped = false;

    while (pros::millis() - start < maxSensorWait && chassis.isInMotion()) {
        if (clampSensor.get() <= 72) {
            std::cout << "Clamp trigger at: " << clampSensor.get() << "mm\n";
            clamped = true;
            doinker.set_value(true);
            break;
        }
        pros::delay(10);
    }

    // Stop forward motion
    chassis.cancelMotion();

    // Clamp regardless of whether sensor triggered
    chassis.lateralSettings.slew = 2;

    // Back out regardless
    driveBackHoldAngleRed(80, 95, -30, 10000);

    chassis.moveToPoint(chassis.getPose().x - 5, chassis.getPose().y, 650, {.forwards = false, .maxSpeed = 90});
    chassis.waitUntilDone();

    chassis.turnToPoint(-15, -48, 500, {.forwards = false, .maxSpeed = 100});
    chassis.waitUntilDone();

    chassis.moveToPoint(-15, -48, 1200, {.forwards = false, .maxSpeed = 80});
    pros::delay(1100);
    clamp.set_value(true);
    chassis.waitUntilDone();
    intake.setState(lib::IntakeState::In);


   chassis.moveToPoint(-30, -55, 1000, {.forwards = true, .maxSpeed = 100});
    chassis.waitUntilDone();

    chassis.turnToHeading(chassis.getPose().theta - 180, 850, {.maxSpeed = 70});
    chassis.waitUntilDone();
    clamp.set_value(false);
        intake.setState(lib::IntakeState::Idle);


    chassis.moveToPoint(-24, -48, 750, {.forwards = true, .maxSpeed = 100});
    chassis.waitUntilDone();

    chassis.turnToPoint(-24, -24, 500, {.forwards = false});
    chassis.waitUntilDone();

    chassis.moveToPoint(-24, -24, 1100, {.forwards = false, .maxSpeed = 70});
    pros::delay(1000);
    clamp.set_value(true);
    chassis.waitUntilDone();
        intake.setState(lib::IntakeState::In);

    chassis.moveToPoint(-55, -55, 1500, {.maxSpeed = 100});
    chassis.waitUntilDone();

   pros::delay(500);

chassis.moveToPoint(-64, -64, 850, {.maxSpeed = 60});
chassis.waitUntilDone();

chassis.moveToPoint(-68, -68, 350, {.maxSpeed = 60});
chassis.waitUntilDone();

chassis.moveToPoint(-48, -48, 750, {.forwards = false, .maxSpeed = 110});
chassis.waitUntilDone();

chassis.moveToPoint(-58,-58, 850, {.maxSpeed = 90});
chassis.waitUntilDone();

chassis.moveToPoint(-48, -48, 500, {.forwards = false, .maxSpeed = 100});
chassis.waitUntilDone();

rightArm.set_value(true);
chassis.moveToPoint(-58,-58, 850, {.maxSpeed = 90});
chassis.waitUntilDone();

chassis.turnToPoint(-48, 0, 750, {.maxSpeed = 60});
pros::delay(650);
rightArm.set_value(false);
chassis.waitUntilDone();

chassis.turnToPoint(0,0, 500, {.maxSpeed = 70});
chassis.waitUntilDone();



}



//BLUE
inline void blueSAWP(){
chassis.setPose(55, 16, 180);
//Score Alliance Stake

chassis.moveToPose(60.5, 11.5, 141, 550, {.horizontalDrift = 10, .maxSpeed = 110});
lift.setState(lib::LiftState::AllianceMid);
chassis.waitUntilDone();

lift.setState(lib::LiftState::AllianceScore);
pros::delay(150);

chassis.moveToPoint(54, 16, 350, {.forwards = false, .maxSpeed = 110});
chassis.waitUntilDone();


//Clamp Goal

chassis.moveToPoint(24, 24, 1150, {.forwards = false, .maxSpeed = 87.5});
pros::delay(500);
lift.setState(lib::LiftState::AllianceScore);
pros::delay(650);
clamp.set_value(true);
chassis.waitUntilDone();
lift.setState(lib::LiftState::Stored);
//Score 3 Rings 

//score rings
chassis.turnToPoint(9.5,37,350, {.maxSpeed = 100});
chassis.waitUntilDone();

intake.setState(lib::IntakeState::In);

chassis.moveToPoint(9.5, 37, 850, {.maxSpeed = 80});
chassis.waitUntilDone();

pros::delay(150);

chassis.moveToPoint(8, 57, 750, {.maxSpeed = 70});
chassis.waitUntilDone();

//protected ring
chassis.moveToPoint(24, 24, 750, {.forwards = false, .maxSpeed = 110});
chassis.waitUntilDone();

chassis.turnToPoint(24, 44, 250);
chassis.waitUntilDone();

chassis.moveToPoint(24, 44, 850, {.maxSpeed = 110});
chassis.waitUntilDone();

//Get Next Ring
chassis.moveToPoint(24, 24, 750, {.forwards = false, .maxSpeed = 110});
chassis.waitUntilDone();

chassis.turnToPoint(48, 0, 500, {.maxSpeed = 100});
chassis.waitUntilDone();

leftArm.set_value(true);
intake.setJamEnabled(false);

//intake.setState(lib::IntakeState::SpinUntil);
intake.setState(lib::IntakeState::In);

chassis.moveToPoint(48, 0, 1000, {.maxSpeed = 70});
//pros::delay(650);
//clamp.set_value(false);
chassis.waitUntilDone();
leftArm.set_value(false);

chassis.moveToPoint(40, 10, 500, {.forwards = false, .maxSpeed = 110});
chassis.waitUntilDone();

chassis.moveToPoint(60, -48, 1300, {.forwards = true, .maxSpeed = 110});
pros::delay(1050);
intake.setState(lib::IntakeState::SpinUntil);
chassis.waitUntilDone();

chassis.moveToPoint(50, -28, 500, {.forwards = false, .maxSpeed = 110});
chassis.waitUntilDone();
clamp.set_value(false);

chassis.moveToPoint(58, -40, 500, {.forwards = true, .maxSpeed = 110});
chassis.waitUntilDone();

//Clamp 2nd Mogo
chassis.turnToPoint(24, -24, 250, {.forwards=false, .maxSpeed = 100});
chassis.waitUntilDone();


chassis.moveToPoint(24, -24, 1250, {.forwards = false, .maxSpeed = 87.5});
pros::delay(1200);
clamp.set_value(true);
chassis.waitUntilDone();

//One ring
chassis.turnToPoint(24, -48, 350);
chassis.waitUntilDone();

intake.setState(lib::IntakeState::In);

chassis.moveToPoint(24, -46, 750, {.maxSpeed = 110});
chassis.waitUntilDone();

//bar touch

chassis.moveToPoint(24, -24, 750, { .forwards = false, .maxSpeed = 110});
chassis.waitUntilDone();

chassis.moveToPoint(11, -11, 1000, {.forwards = true, .maxSpeed = 110});
lift.setState(lib::LiftState::Recieve);
chassis.waitUntilDone();

}

inline void blue7Ring(){
chassis.setPose(55, 16, 180);
//Score Alliance Stake

chassis.moveToPose(60.5, 11.5, 141, 550, {.horizontalDrift = 10, .maxSpeed = 110});
lift.setState(lib::LiftState::AllianceMid);
chassis.waitUntilDone();

lift.setState(lib::LiftState::AllianceScore);
pros::delay(150);

chassis.moveToPoint(54, 16, 350, {.forwards = false, .maxSpeed = 110});
chassis.waitUntilDone();


//Clamp Goal

chassis.moveToPoint(24, 24, 1150, {.forwards = false, .maxSpeed = 87.5});
pros::delay(500);
lift.setState(lib::LiftState::AllianceScore);
pros::delay(650);
clamp.set_value(true);
chassis.waitUntilDone();
lift.setState(lib::LiftState::Stored);
//Score 3 Rings 

//score rings
chassis.turnToPoint(9,37,350, {.maxSpeed = 100});
chassis.waitUntilDone();

intake.setState(lib::IntakeState::In);

chassis.moveToPoint(9, 37, 850, {.maxSpeed = 80});
chassis.waitUntilDone();

pros::delay(150);

chassis.moveToPoint(8, 56, 750, {.maxSpeed = 70});
chassis.waitUntilDone();

//protected ring
chassis.moveToPoint(24, 24, 750, {.forwards = false, .maxSpeed = 110});
chassis.waitUntilDone();

chassis.turnToPoint(24, 44, 250);
chassis.waitUntilDone();

chassis.moveToPoint(24, 44, 850, {.maxSpeed = 110});
chassis.waitUntilDone();
//Get Next Ring
chassis.moveToPoint(24, 24, 1000, {.forwards = false, .maxSpeed = 110});
chassis.waitUntilDone();

//ring stack
chassis.turnToPoint(48, 48, 500);
chassis.waitUntilDone();

chassis.moveToPoint(55, 55, 1000, {.maxSpeed = 100});
chassis.waitUntilDone();

chassis.moveToPoint(64, 64, 750, {.maxSpeed = 60});
chassis.waitUntilDone();

chassis.moveToPoint(46, 46, 750, {.forwards = false, .maxSpeed = 110});
chassis.waitUntilDone();

chassis.moveToPoint(60, 60, 1000, {.maxSpeed = 90});
chassis.waitUntilDone();

chassis.moveToPoint(48, 48, 1000, {.forwards = false, .maxSpeed = 100});
chassis.waitUntilDone();

//middle ring stack alliance
chassis.turnToPoint(48, 0, 500, {.maxSpeed = 100});
chassis.waitUntilDone();

leftArm.set_value(true);
intake.setJamEnabled(false);

chassis.moveToPoint(48, 0, 1000, {.maxSpeed = 90});
chassis.waitUntilDone();

chassis.moveToPoint(45, 3, 1000, {.maxSpeed = 90});
chassis.waitUntilDone();
leftArm.set_value(false);

chassis.moveToPoint(64, -64, 5000, {.maxSpeed = 127});
}

inline void blue6RingNegative(){
chassis.setPose(55, 16, 180);
//Score Alliance Stake

chassis.moveToPose(60.5, 11.5, 141, 550, {.horizontalDrift = 10, .maxSpeed = 110});
lift.setState(lib::LiftState::AllianceMid);
chassis.waitUntilDone();

lift.setState(lib::LiftState::AllianceScore);
pros::delay(150);

chassis.moveToPoint(54, 16, 350, {.forwards = false, .maxSpeed = 110});
chassis.waitUntilDone();


//Clamp Goal

chassis.moveToPoint(24, 24, 1150, {.forwards = false, .maxSpeed = 87.5});
pros::delay(500);
lift.setState(lib::LiftState::AllianceScore);
pros::delay(650);
clamp.set_value(true);
chassis.waitUntilDone();
lift.setState(lib::LiftState::Stored);
//Score 3 Rings 

//score rings
chassis.turnToPoint(9,37,350, {.maxSpeed = 100});
chassis.waitUntilDone();

intake.setState(lib::IntakeState::In);

chassis.moveToPoint(9, 37, 850, {.maxSpeed = 80});
chassis.waitUntilDone();

pros::delay(150);

chassis.moveToPoint(8, 56, 750, {.maxSpeed = 70});
chassis.waitUntilDone();

//protected ring
chassis.moveToPoint(24, 24, 750, {.forwards = false, .maxSpeed = 110});
chassis.waitUntilDone();

chassis.turnToPoint(24, 44, 250);
chassis.waitUntilDone();

chassis.moveToPoint(24, 44, 850, {.maxSpeed = 110});
chassis.waitUntilDone();
//Get Next Ring
chassis.moveToPoint(24, 24, 1000, {.forwards = false, .maxSpeed = 110});
chassis.waitUntilDone();

//ring stack
chassis.turnToPoint(48, 48, 500);
chassis.waitUntilDone();

chassis.moveToPoint(55, 55, 1000, {.maxSpeed = 100});
chassis.waitUntilDone();

chassis.moveToPoint(64, 64, 750, {.maxSpeed = 60});
chassis.waitUntilDone();

chassis.moveToPoint(46, 46, 750, {.forwards = false, .maxSpeed = 110});
chassis.waitUntilDone();

chassis.moveToPoint(60, 60, 1000, {.maxSpeed = 90});
chassis.waitUntilDone();

chassis.moveToPoint(48, 48, 1000, {.forwards = false, .maxSpeed = 100});
chassis.waitUntilDone();

//middle ring stack alliance
chassis.turnToPoint(0, 0, 750, {.maxSpeed = 100});
chassis.waitUntilDone();

chassis.moveToPoint(10, 10, 1000, {.maxSpeed = 80});
lift.setState(lib::LiftState::Recieve);
chassis.waitUntilDone();
}

inline void blue6RingPositive(){
chassis.setPose(55, -16, 0);
//Score Alliance Stake

chassis.moveToPose(60, -12, 41, 650, {.horizontalDrift = 10, .maxSpeed = 110});
//lift.setState(lib::LiftState::AllianceMid);
chassis.waitUntilDone();

//lift.setState(lib::LiftState::AllianceScore);
pros::delay(150);

chassis.moveToPoint(54, -16, 350, {.forwards = false, .maxSpeed = 110});
chassis.waitUntilDone();

//Clamp Goal
chassis.moveToPoint(24, -24, 1150, {.forwards = false, .maxSpeed = 90});
pros::delay(500);
//lift.setState(lib::LiftState::AllianceScore);
pros::delay(500);
clamp.set_value(true);
chassis.waitUntilDone();
//lift.setState(lib::LiftState::Stored);

chassis.turnToPoint(24, -44, 500);
chassis.waitUntilDone();

intake.setState(lib::IntakeState::In);

chassis.moveToPoint(24, -44, 850, {.maxSpeed = 110});
chassis.waitUntilDone();

//Get Next Ring
chassis.moveToPoint(24, -24, 1000, {.forwards = false, .maxSpeed = 110});
chassis.waitUntilDone();

//ring stack
chassis.turnToPoint(48, -48, 500);
chassis.waitUntilDone();

chassis.moveToPoint(48, -48, 1000, {.maxSpeed = 100});
chassis.waitUntilDone();

chassis.moveToPoint(67, -67, 1150, {.maxSpeed = 60});
chassis.waitUntilDone();

chassis.moveToPoint(48, -48, 750, {.forwards = false, .maxSpeed = 110});
chassis.waitUntilDone();

chassis.moveToPoint(58,-58, 1000, {.maxSpeed = 90});
chassis.waitUntilDone();

chassis.moveToPoint(48, -48, 1000, {.forwards = false, .maxSpeed = 100});
chassis.waitUntilDone();

//middle ring stack alliance
chassis.turnToPoint(48, 0, 500, {.maxSpeed = 100});
chassis.waitUntilDone();

leftArm.set_value(true);
intake.setJamEnabled(false);

chassis.moveToPoint(48, 0, 1250, {.maxSpeed = 80});
pros::delay(1000);
chassis.waitUntilDone();
leftArm.set_value(false);
pros::delay(750);

chassis.turnToPoint(24, -24, 500, {.maxSpeed = 100});
chassis.waitUntilDone();

chassis.moveToPoint(24, -24, 850, {.maxSpeed = 100});
chassis.waitUntilDone();

chassis.turnToPoint(0, 0, 500, {.maxSpeed = 100});
chassis.waitUntilDone();

chassis.moveToPoint(12, -12, 1000, {.maxSpeed = 60});
lift.setState(lib::LiftState::Recieve);
chassis.waitUntilDone();
}

inline void blueGoalRush(){
    chassis.setPose(58, -48, 275);
    leftArm.set_value(false);
    chassis.lateralSettings.slew = 0;

    intake.setState(lib::IntakeState::SpinUntil);
    rightArm.set_value(true);

    chassis.moveToPoint(18, -43, 1400, {.forwards = true, .maxSpeed = 127});

    const int maxSensorWait = 1500; // 1.5 second cutoff
    int start = pros::millis();
    bool clamped = false;

    while (pros::millis() - start < maxSensorWait && chassis.isInMotion()) {
        if (clampSensor.get() <= 72) {
            std::cout << "Clamp trigger at: " << clampSensor.get() << "mm\n";
            clamped = true;
            doinker.set_value(true);
            break;
        }
        pros::delay(10);
    }

    // Stop forward motion
    chassis.cancelMotion();

    // Clamp regardless of whether sensor triggered
    chassis.lateralSettings.slew = 2;

    // Back out regardless
    driveBackHoldAngle(80, 275, 30, 10000);

    chassis.moveToPoint(chassis.getPose().x + 5, chassis.getPose().y, 650, {.forwards = false, .maxSpeed = 90});
    chassis.waitUntilDone();

    chassis.turnToPoint(15, -48, 500, {.forwards = false, .maxSpeed = 100});
    chassis.waitUntilDone();

    chassis.moveToPoint(15, -48, 1200, {.forwards = false, .maxSpeed = 80});
    pros::delay(1100);
    clamp.set_value(true);
    chassis.waitUntilDone();
    intake.setState(lib::IntakeState::In);


   chassis.moveToPoint(30, -55, 1000, {.forwards = true, .maxSpeed = 100});
    chassis.waitUntilDone();

    chassis.turnToHeading(chassis.getPose().theta - 180, 850, {.maxSpeed = 70});
    chassis.waitUntilDone();
    clamp.set_value(false);
        intake.setState(lib::IntakeState::Idle);


    chassis.moveToPoint(24, -48, 750, {.forwards = true, .maxSpeed = 100});
    chassis.waitUntilDone();

    chassis.turnToPoint(24, -24, 500, {.forwards = false});
    chassis.waitUntilDone();

    chassis.moveToPoint(24, -24, 1100, {.forwards = false, .maxSpeed = 70});
    pros::delay(1000);
    clamp.set_value(true);
    chassis.waitUntilDone();
        intake.setState(lib::IntakeState::In);

    chassis.moveToPoint(55, -55, 1500, {.maxSpeed = 100});
    chassis.waitUntilDone();

   pros::delay(500);

chassis.moveToPoint(64, -64, 850, {.maxSpeed = 60});
chassis.waitUntilDone();

chassis.moveToPoint(68, -68, 350, {.maxSpeed = 60});
chassis.waitUntilDone();

chassis.moveToPoint(48, -48, 750, {.forwards = false, .maxSpeed = 110});
chassis.waitUntilDone();

chassis.moveToPoint(58,-58, 850, {.maxSpeed = 90});
chassis.waitUntilDone();

chassis.moveToPoint(48, -48, 500, {.forwards = false, .maxSpeed = 100});
chassis.waitUntilDone();

rightArm.set_value(true);
chassis.moveToPoint(58,-58, 850, {.maxSpeed = 90});
chassis.waitUntilDone();

chassis.turnToPoint(0, -48, 750, {.maxSpeed = 70});
pros::delay(650);
rightArm.set_value(false);
chassis.waitUntilDone();

chassis.turnToPoint(0,0, 500, {.maxSpeed = 70});
chassis.waitUntilDone();

}


/*
//SKILLS
inline void stateSkills(){

 ///////////////////////////////////////////////
  //--------------FIRST SECTION----------------//
  ///////////////////////////////////////////////

  chassis.setPose(-61.5, 0, 90);

  //-----------ALLIANCE STAKE------------------//

  intake.setState(lib::IntakeState::In);
  pros::delay(200);
  intake.setState(lib::IntakeState::Idle);

  //-----------CLAMP FIRST MOGO------------------//

  chassis.moveToPoint(-48, 0, 850, {.forwards = true, .maxSpeed = 100});
  chassis.waitUntilDone();

  chassis.turnToPoint(-48, -24, 450);
  chassis.waitUntilDone();

  chassis.moveToPoint(-48, 24, 1000, {.forwards=false, .maxSpeed = 70});
  chassis.waitUntilDone();

  clamp.set_value(true);

//-----------SCORE ONE RING------------------//

  chassis.turnToPoint(-32, 24, 450);
  chassis.waitUntilDone();

  intake.setState(lib::IntakeState::In);

  chassis.moveToPoint(-32, 24, 750, {.forwards=true, .maxSpeed = 100});
  chassis.waitUntilDone();


  //-----------OBTAIN 2nd RING------------------//

  //chassis.turnToPoint(24, 48, 500);
  //chassis.waitUntilDone();

  chassis.moveToPoint(24, 48, 1750, {.forwards=true, .maxSpeed = 100});
  pros::delay(1000);
  lift.setState(lib::LiftState::Recieve);
  chassis.waitUntilDone();

  chassis.moveToPoint(0, 44, 1000, {.forwards=false, .maxSpeed = 100});
  chassis.waitUntilDone();
  lift.setState(lib::LiftState::Mid);
  intake.setState(lib::IntakeState::Idle);
  pros::delay(100);
  intake.setState(lib::IntakeState::In);

  //-----------LADY BROWN 1------------------//

  chassis.turnToPoint(0, 63, 500);
  chassis.waitUntilDone();
 
  chassis.moveToPoint(0, 63, 1000, {.forwards=true, .maxSpeed =70});
  chassis.waitUntilDone();

  chassis.setPose(0,61, chassis.getPose().theta);
  pros::delay(50);
  lift.setState(lib::LiftState::Score);
  pros::delay(450);

  //-----------SCORE THREE RINGS------------------//

  chassis.moveToPoint(0, 46, 750, {.forwards=false, .maxSpeed = 100});
  chassis.waitUntilDone();
  lift.setState(lib::LiftState::Stored);


  chassis.turnToPoint(-33, 46, 500);
  chassis.waitUntilDone();

  chassis.moveToPoint(-33, 46, 1000, {.forwards=true, .maxSpeed = 90});
  chassis.waitUntilDone();

  chassis.moveToPoint(-60, 46, 1750, {.forwards=true, .maxSpeed = 60});
  chassis.waitUntilDone();

 // chassis.turnToPoint(-49, 57, 500);
  //chassis.waitUntilDone();

  chassis.moveToPoint(-43, 60, 1000, {.forwards=true, .maxSpeed =80});
  chassis.waitUntilDone();

  //-----------Place in corner------------------//

  //chassis.turnToPoint(-59, 59, 500, {.forwards = false});
 // chassis.waitUntilDone();

  chassis.moveToPoint(-61, 61, 1000, {.forwards=false, .maxSpeed =60});
  chassis.waitUntilDone();

  clamp.set_value(false);
  intake.setState(lib::IntakeState::Idle);


  chassis.moveToPoint(-48, 48, 750, {.forwards=true, .maxSpeed =100});
  chassis.waitUntilDone();
  
  //RESET//
  chassis.turnToHeading(90, 500);
  chassis.waitUntilDone();

  resetRobotPos(leftSensor, "positive_y");
  resetRobotPos(backSensor, "negative_x");

    intake.setState(lib::IntakeState::In);
  lift.setState(lib::LiftState::Recieve);

  ///////////////////////////////////////////////
  //--------------2ND SECTION----------------//
  ///////////////////////////////////////////////

  //Get ring for alliance stake

  chassis.moveToPoint(40, 48, 1500, {.forwards=true, .maxSpeed =100});
  chassis.waitUntilDone();
  rightArm.set_value(true);


  chassis.turnToHeading(90,500);
  chassis.waitUntilDone();

  resetRobotPos(leftSensor, "positive_y");
  resetRobotPos(frontSensor, "positive_x");



  chassis.moveToPoint(48, 48, 1250, {.forwards=true, .maxSpeed =100});
  chassis.waitUntilDone();

  //Clamp second goal

  chassis.turnToPoint(61, 22, 500, {.forwards = false});
    intake.setState(lib::IntakeState::In);

  chassis.waitUntilDone();
 rightArm.set_value(false);


  chassis.moveToPoint(61, 22, 1000, {.forwards=false, .maxSpeed =90});
  chassis.waitUntilDone();
  clamp.set_value(true);

  //put in corner
  chassis.turnToPoint(64, 54, 500);
  lift.setState(lib::LiftState::Mid);
  pros::delay(150);
  intake.setState(lib::IntakeState::Out);
  chassis.waitUntilDone();
   rightArm.set_value(true);


  chassis.moveToPoint(64, 54, 1000, {.forwards=true, .maxSpeed =100});
  chassis.waitUntilDone();

  chassis.turnToHeading(270, 850, {.maxSpeed = 70});
  pros::delay(500);
  rightArm.set_value(false);
  chassis.waitUntilDone();
  pros::delay(150);

  chassis.turnToPoint(64, 64, 500, {.forwards=false, .maxSpeed = 70});
  chassis.waitUntilDone();

  chassis.moveToPoint(64, 64, 500, {.forwards=false, .maxSpeed =50});
  chassis.waitUntilDone();
    clamp.set_value(false);


  intake.setState(lib::IntakeState::In);



  //get third goal
  intake.setState(lib::IntakeState::In);

  chassis.moveToPoint(48, 30, 1150, {.forwards=true, .maxSpeed =100});
  chassis.waitUntilDone();

  chassis.turnToHeading(0, 650);
  chassis.waitUntilDone();
  resetRobotPos(rightSensor, "positive_x");
  resetRobotPos(frontSensor, "positive_y");
 

  chassis.moveToPoint(48, 0, 1150, {.forwards=false, .maxSpeed =70});
  intake.setState(lib::IntakeState::Idle);
  chassis.waitUntilDone();
  resetRobotPos(rightSensor, "positive_x");
  resetRobotPos(frontSensor, "positive_y");
  clamp.set_value(true);


  //score alliance stake

  chassis.turnToPoint(63, 0, 500, {.forwards=true});
  chassis.waitUntilDone();
   resetRobotPos(leftSensor, "positive_y");
  resetRobotPos(frontSensor, "positive_x");

  chassis.moveToPoint(63, 0, 1000, {.forwards=true, .maxSpeed =60});
  chassis.waitUntilDone();
  chassis.setPose(61, 0, chassis.getPose().theta);

  chassis.moveToPoint(57, 0, 500, {.forwards=false, .maxSpeed =80});
  chassis.waitUntilDone();
  lift.setState(lib::LiftState::Alliance);

  chassis.moveToPoint(54, 0, 500, {.forwards=false, .maxSpeed =60});
  chassis.waitUntilDone();

  lift.setState(lib::LiftState::AllianceScore);
  pros::delay(250); 

  chassis.moveToPoint(48, 0, 500, {.forwards=false, .maxSpeed =80});
  pros::delay(250);
  intake.setState(lib::IntakeState::In);
  lift.setState(lib::LiftState::Stored);
  chassis.waitUntilDone();

  chassis.turnToHeading(0, 500);
  chassis.waitUntilDone();
  resetRobotPos(frontSensor, "positive_y");
  resetRobotPos(rightSensor, "positive_x");

  chassis.moveToPoint(48, 24, 900, {.forwards=true, .maxSpeed =90});
  chassis.waitUntilDone();

  chassis.turnToHeading(270, 500);
  chassis.waitUntilDone();
  resetRobotPos(rightSensor, "positive_y");
  resetRobotPos(backSensor, "positive_x");

  ///////////////////////////////////////////////
  //--------------3RD SECTION----------------//
  ///////////////////////////////////////////////

  //score one ring


  chassis.moveToPoint(24, 24, 850, {.forwards=true, .maxSpeed =90});
  chassis.waitUntilDone();

  chassis.turnToHeading(270, 500);
  chassis.waitUntilDone();
  resetRobotPos(rightSensor, "positive_y");
  resetRobotPos(backSensor, "positive_x");


//cross under ladder score 3
  chassis.turnToPoint(-15, -15, 500);
  chassis.waitUntilDone();
  intake.setState(lib::IntakeState::Idle);

  chassis.moveToPoint(-15, -15, 1350, {.forwards=true, .maxSpeed =90});
  chassis.waitUntilDone();
  intake.setState(lib::IntakeState::In);


  chassis.moveToPoint(-48, -48, 1650, {.forwards=true, .maxSpeed =70});
  chassis.waitUntilDone();

  //score two and corner
  //chassis.turnToPoint(-62, -48, 500);
  chassis.turnToHeading(270, 500);
  chassis.waitUntilDone();
  resetRobotPos(leftSensor, "negative_y");
  resetRobotPos(frontSensor, "negative_x");


  chassis.moveToPoint(-63, -48, 850, {.forwards=true, .maxSpeed =80});
  chassis.waitUntilDone();


  //chassis.turnToPoint(-45, -62, 500);
  //chassis.waitUntilDone();

  chassis.moveToPoint(-50, -60, 1000, {.forwards=true, .maxSpeed =100});
  chassis.waitUntilDone();

  //-----------Place in corner------------------//

 // chassis.turnToPoint(-56, -62, 500, {.forwards 1d`s= false});
 // chassis.waitUntilDone();

  chassis.moveToPoint(-60, -62, 750, {.forwards=false, .maxSpeed =100});
  chassis.waitUntilDone();


  linearController.kD = 3;
  angularController.kD = 19;
  
  //chassis.turnToPoint(-48, -48, 500);
  //chassis.waitUntilDone();
  clamp.set_value(false);
  intake.setState(lib::IntakeState::Idle);



  chassis.moveToPoint(-48, -48, 750, {.forwards=true, .maxSpeed =100});
  chassis.waitUntilDone();
  
  //RESET//
  chassis.turnToHeading(180, 500);
  chassis.waitUntilDone();

///////////////////////////////////////////////
  //--------------4th SECTION----------------//
  ///////////////////////////////////////////////

  //clamp and one ring
  resetRobotPos(frontSensor, "negative_y");
  resetRobotPos(rightSensor, "negative_x");
  
  chassis.turnToPoint(-48, -24, 500, {.forwards = false});
  chassis.waitUntilDone();

  chassis.moveToPoint(-48, -24, 1150, {.forwards=false, .maxSpeed =70});
  chassis.waitUntilDone();
  clamp.set_value(true);
  lift.setState(lib::LiftState::Recieve);

  chassis.turnToPoint(-24, -48, 500);
  chassis.waitUntilDone();
  intake.setState(lib::IntakeState::In);

  chassis.moveToPoint(-24, -48, 1000, {.forwards=true, .maxSpeed =100});
  chassis.waitUntilDone();

//lady brown stuff

  chassis.turnToPoint(0, -45, 500, {.forwards = true});
  chassis.waitUntilDone();

  chassis.moveToPoint(0, -45, 850, {.forwards=true, .maxSpeed = 100});
  chassis.waitUntilDone();
  lift.setState(lib::LiftState::Mid);
  intake.setState(lib::IntakeState::Idle);
  pros::delay(100);
  intake.setState(lib::IntakeState::In);

  //-----------LADY BROWN 1------------------//

  chassis.turnToPoint(0, -63, 500);
  chassis.waitUntilDone();
 
  chassis.moveToPoint(0, -63, 850, {.forwards=true, .maxSpeed =80});
  chassis.waitUntilDone();

  chassis.setPose(0,-61, chassis.getPose().theta);
  pros::delay(50);
  lift.setState(lib::LiftState::Score);
  pros::delay(350);

  chassis.moveToPoint(0, -48, 750, {.forwards=false, .maxSpeed = 100});
  chassis.waitUntilDone();
  lift.setState(lib::LiftState::Stored);

  //now score rings

  intake.setState(lib::IntakeState::In);

  chassis.turnToPoint(24, -48, 500);
  chassis.waitUntilDone();

  chassis.moveToPoint(24, -48, 1000, {.forwards=true, .maxSpeed =100});
  chassis.waitUntilDone();
  resetRobotPos(rightSensor, "negative_y");
  resetRobotPos(frontSensor, "positive_x");


  chassis.turnToHeading(0, 500);
  chassis.waitUntilDone();
  resetRobotPos(backSensor, "negative_y");
  resetRobotPos(rightSensor, "positive_x");


  chassis.moveToPoint(24, -24, 1150, {.forwards=true, .maxSpeed =100});
  chassis.waitUntilDone();

        intake.sort_override = true;



  chassis.turnToPoint(44, -44, 500);
  chassis.waitUntilDone();

  chassis.moveToPoint(44, -44, 850, {.forwards=true, .maxSpeed =100});
  intake.setState(lib::IntakeState::In);
  chassis.waitUntilDone();



  //score two and corner
  //chassis.turnToPoint(60, -48, 500);
  //chassis.waitUntilDone();

  chassis.moveToPoint(60, -48, 750, {.forwards=true, .maxSpeed =70});
  chassis.waitUntilDone();

 chassis.moveToPoint(24, -48, 850, {.forwards=false, .maxSpeed =100});
 chassis.waitUntilDone();

 // chassis.turnToPoint(48, -56, 500);
  //chassis.waitUntilDone();
  //rightArm.set_value(true);

  chassis.moveToPoint(48, -60, 500, {.forwards=true, .maxSpeed =100});
  rightArm.set_value(true);
  chassis.waitUntilDone();

  rightArm.set_value(true);

  chassis.moveToPoint(55, -63, 650, {.forwards=true, .maxSpeed =100});
  chassis.waitUntilDone();



chassis.turnToPoint(62, -63, 1450, {.forwards = false,.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 65});
pros::delay(1250);
  intake.setState(lib::IntakeState::Idle);

chassis.waitUntilDone();
    rightArm.set_value(false);


  chassis.moveToPoint(64, -63, 500, {.forwards=false, .maxSpeed =127});
  chassis.waitUntilDone();
    clamp.set_value(false);

  //HANGGG
  chassis.moveToPoint(24, -24, 500, {.forwards=true, .maxSpeed =127});
  chassis.waitUntilDone();

  chassis.turnToPoint(64, -64, 500);
  chassis.waitUntilDone();
  lift.setState(lib::LiftState::Score);

  chassis.moveToPoint(0, -0, 2000, {.forwards=false, .maxSpeed =127});
  chassis.waitUntilDone();
}

*/