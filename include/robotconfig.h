#pragma once
#include "lemlib/api.hpp"
#include "lemlib/asset.hpp"
#include "main.h"
#include "lib/intake.hpp"
#include "lib/lights.hpp"
#include "lib/lift.hpp"
#include "lib/liftpid.h"
#include "pros/adi.hpp"


inline bool armLoading = false;
enum class team {red, blue, skills, none};
inline team teamColor = team::none;
inline int current_auto = 0;

inline static bool isAlliance = true; 
inline static bool jamCode = true; 
inline static bool isTipping = false;
inline static bool isMid = false;



// controller
inline pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
inline pros::MotorGroup leftMotors({-3,4,-11},
                            pros::MotorGearset::blue); // left motor group - ports 3 (reversed), 4, 5 (reversed)
inline pros::MotorGroup rightMotors({7,8,-9}, pros::MotorGearset::blue); 

// Inertial Sensor on port 10
inline pros::Imu imu(12);

// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
inline pros::Rotation verticalEnc(-2);
inline pros::Rotation horizontalEnc(-1);
// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
inline lemlib::TrackingWheel vertical(&verticalEnc, 2.03, 0);
inline lemlib::TrackingWheel horizontal(&horizontalEnc, 2.03, 0);


// drivetrain settings
inline lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              13.5, // 10 inch track width
                              lemlib::Omniwheel::NEW_275, // using new 4" omnis
                              450, // drivetrain rpm is 360
                              12 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

inline lemlib::ControllerSettings linearController(9, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              5 // maximum acceleration (slew)
);

inline lemlib::ControllerSettings angularController(2.85, // proportional gain (kP)
                                              0.1, //0.3, //integral gain (kI)
                                              19, // derivative gain (kD)
                                              6, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);
// sensors for odometry
inline lemlib::OdomSensors sensors(&vertical, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            &horizontal, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// input curve for throttle input during driver control
inline lemlib::ExpoDriveCurve throttleCurve(0, // joystick deadband out of 127
                                     0, // minimum output where drivetrain will move out of 127
                                     1.0005 // expo curve gain
);

// input curve for steer input during driver control
inline lemlib::ExpoDriveCurve steerCurve(0, // joystick deadband out of 127
                                  0, // minimum output where drivetrain will move out of 127
                                  1.0005 // expo curve gain
);

// create the chassis
inline lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);


inline pros::Optical color(20);
inline pros::Optical color2(20);
inline pros::MotorGroup intakeMotor({-19});
inline pros::Motor intakeMotors(-19, pros::v5::MotorGears::blue);
inline pros::adi::Pneumatics sorter('D', false);
inline lib::Intake intake(&intakeMotor, &color, &color2, &sorter);

inline pros::MotorGroup armMotors({5, 5});
inline lib::Lift lift(&armMotors, intakeMotors, 36.0/36.0, {2, 0, 4});
//inline lib::Lift lift(&armMotors, 12.0 / 36, {2.5, 0, 1.75});

inline pros::adi::Pneumatics clamp('H', false);

inline pros::adi::Pneumatics leftArm('G', false);
inline pros::adi::Pneumatics rightArm('F', false);

inline pros::adi::Pneumatics doinker('E', false);



inline pros::Distance clampSensor(10); // Front sensor port
inline pros::Distance backSensor(10);  // Back sensor port
inline pros::Distance leftSensor(10);  // Left sensor port
inline pros::Distance rightSensor(10);  // Right sensor port
inline pros::Distance frontSensor(10);  // Right sensor port

inline pros::Rotation liftSensor(6);


inline lib::Lights lights = lib::Lights();

ASSET(example_txt)