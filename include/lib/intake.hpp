#pragma once
#include "lib/TaskWrapper.hpp"
#include "lib/StateMachine.hpp"
#include "main.h"
#include "pros/adi.hpp"
#include "pros/optical.hpp"
#include <cstdint>

namespace lib {

// Updated enum: added SpinUntil state.
enum class IntakeState { In, Out, Jam, Idle, SpinUntil };

class Intake : public StateMachine<IntakeState, IntakeState::Idle>, public ryan::TaskWrapper {
private:
  std::shared_ptr<pros::MotorGroup> motors;
  std::shared_ptr<pros::Optical> color;
  // Note: color2 is used in the cpp code even though it's not stored here.
  std::shared_ptr<pros::adi::Pneumatics> sort;
  int sort_time = 0;

  // Used to time the SpinUntil state.
  uint32_t spinUntilStartTime = 0;

public:
  bool sort_override = false;
  // Add a public flag to toggle jam detection.
  bool jamEnabled = true;

  // Setter for jam detection.
  void setJamEnabled(bool enabled) {
    jamEnabled = enabled;
  }

  Intake(pros::MotorGroup *motors, pros::Optical *color, pros::Optical *color2, pros::adi::Pneumatics *sort)
      : motors(motors), color(color), sort(sort) {
    color->set_integration_time(10);
    color->set_led_pwm(100);
    color2->set_led_pwm(100);
    color2->set_integration_time(10);
  }
  void loop() override;
};

}  // namespace lib
