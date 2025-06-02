#include "main.h"
#include "autons.hpp"
#include <string>
#include "lemlib/chassis/chassis.hpp"
#include "lib/intake.hpp"
#include "lib/lift.hpp"
#include "pros/misc.h"
#include "robotconfig.h" 
#include "robodash/api.h"


void on_center_button() {
  static bool pressed = false;
  pressed = !pressed;
  if (pressed) {
    pros::lcd::set_text(2, "I was pressed!");
  } else {
    pros::lcd::clear_line(2);
  }
}


// ------------------------------------------------------------------
// auton selector
// ------------------------------------------------------------------
rd::Selector selector({
    {"(SKILLS) Driving", redSAWP},

    {"(RED) SAWP", redSAWP},
    {"(RED) 7 Ring", red7Ring},
    {"(RED) 6 Ring Neg", red6RingNegative},
    {"(RED) 6 Ring Pos", red6RingPositive},
    {"(RED) Goal Rush", redGoalRush},

    {"(BLUE) SAWP", blueSAWP},
    {"(BLUE) 7 Ring", blue7Ring},
    {"(BLUE) 6 Ring Neg", blue6RingNegative},
    {"(BLUE) 6 Ring Pos", blue6RingPositive},
    {"(BLUE) Goal Rush", blueGoalRush}
});

// ------------------------------------------------------------------
// Decide team color based on auton name
// ------------------------------------------------------------------
team decideColor(const std::string &name) {
  if (name.find("SKILLS") != std::string::npos)   return team::skills;
  else if (name.find("RED")   != std::string::npos) return team::red;
  else if (name.find("BLUE")  != std::string::npos) return team::blue;
  return team::red;
}

// ------------------------------------------------------------------
// Print helper for debugging
// ------------------------------------------------------------------
void printColor(team c) {
  std::cout << "[DEBUG] Team color = "
            << (c == team::red    ? "RED"
             : c == team::blue   ? "BLUE"
             : c == team::skills ? "SKILLS"
                                 : "UNKNOWN")
            << std::endl;
}

// ------------------------------------------------------------------
// Fire the leftArm piston only on Goal Rush
// ------------------------------------------------------------------
void handlePiston(const std::string &name) {         // ◀ new
  if (name.find("Goal Rush") != std::string::npos) {
    leftArm.set_value(true);
  } else {
    leftArm.set_value(false);
  }
}                                                    // ◀ end new

std::string current_auto_name;

// ------------------------------------------------------------------
// Set robot pose based on selected auton
// ------------------------------------------------------------------
void resetRobotPose(const std::string &autoName) {
  if      (autoName.find("SKILLS")    != std::string::npos) chassis.setPose({   0.0,  48.0,   0.0});
  else if (autoName.find("RED")       != std::string::npos &&
           autoName.find("SAWP")      != std::string::npos) chassis.setPose({ -36.0,  36.0,  90.0});
  else if (autoName.find("BLUE")      != std::string::npos &&
           autoName.find("SAWP")      != std::string::npos) chassis.setPose({  36.0,  36.0, -90.0});
  else if (autoName.find("RED")       != std::string::npos &&
           autoName.find("7 Ring")   != std::string::npos) chassis.setPose({ -48.0,  24.0,   0.0});
  else if (autoName.find("BLUE")      != std::string::npos &&
           autoName.find("7 Ring")   != std::string::npos) chassis.setPose({  48.0,  24.0,   0.0});
  // …and so on…
}

// ------------------------------------------------------------------
// Read initial auton, set color, pose, and piston
// ------------------------------------------------------------------
void auton_check_first() {
  FILE* file = fopen("auto.txt", "r");
  if (file) {
    char buf[128] = {0};
    fread(buf, 1, sizeof(buf)-1, file);
    fclose(file);
    current_auto_name = buf;
  } else {
    current_auto_name = "(RED) SAWP";
  }

  teamColor = decideColor(current_auto_name);
  resetRobotPose(current_auto_name);
  handlePiston(current_auto_name);                // ◀ new

  std::cout << "[auton_check_first] Loaded name: " << current_auto_name << "\n";
  printColor(teamColor);
}

// ------------------------------------------------------------------
// Watch for selector changes, update pose, color, and piston
// ------------------------------------------------------------------
void auton_check_loop() {
  pros::Task{[=] {
    while (true) {
      auto newName = selector.get_auton()->name;
      if (newName != current_auto_name) {
        current_auto_name = newName;
        teamColor         = decideColor(current_auto_name);
        resetRobotPose(current_auto_name);
        handlePiston(current_auto_name);            // ◀ new

        std::cout << "[auton_check_loop] New selection: " << current_auto_name << "\n";
        printColor(teamColor);
      }
      pros::delay(100);
    }
  }};
}


void initialize() {
auton_check_first();
auton_check_loop();
chassis.calibrate();

lift.startTask();
intake.startTask();
/*
pros::lcd::initialize(); // initialize brain screen


    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs

    // thread to for brain screen and position logging
    pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);
        }
    });
  //lights.startTask();*/
}



void disabled() {}



void competition_initialize() {}



void autonomous() {  
lift.startTask();
intake.startTask();
intake.setJamEnabled(true);

 // redStateSAWP();
  selector.run_auton();
  //bluePosRush();
//stateSkills();
//redStateSAWP();
}


void opcontrol() {
  intake.setJamEnabled(false);
  lift.startTask();
  intake.startTask();


  lights.startTimer();
  float liftTarget = -1;

  leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

  lift.setState(lib::LiftState::Stored);

  while (true) {

    chassis.arcade(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y),
                   controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X), true);
    lib::IntakeState newState =
        (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
            ? lib::IntakeState::In
        : (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
            ? lib::IntakeState::Out
            : lib::IntakeState::Idle;

    if (intake.getState() != newState) {
      intake.setState(newState);
    }
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
      intake.sort_override = true;
    } else {
      intake.sort_override = false;
    }
if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
  isTipping = !isTipping;
  lift.setState(isTipping 
    ? lib::LiftState::Tipping 
    : lib::LiftState::Stored
  );
}
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
      clamp.toggle();
    }

if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
  isMid = !isMid;
  lift.setState(
    isMid 
      ? lib::LiftState::Mid 
      : lib::LiftState::Stored
  );
}
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
      lift.startTask();
      intake.startTask();
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
      lift.setState(lib::LiftState::Alliance);
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
      lift.itterateState(1);
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
      //lift.setState(lib::LiftState::Tipping);
      rightArm.toggle();
    }

    pros::delay(15);
  }
}

