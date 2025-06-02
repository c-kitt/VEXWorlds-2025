#include "lib/intake.hpp"
#include "pros/rtos.hpp"
#include "pros/optical.hpp"
#include "pros/misc.h"
#include "pros/adi.hpp"
#include "robotconfig.h"
#include "main.h"

using namespace lib;

void Intake::loop() {
  int jamTimer = 0;
  uint32_t now;
  uint32_t jamStartTime = 0;

  while (true) {
    // Jam detection
    if (jamEnabled) {
      if (std::abs(motors->get_actual_velocity_all()[0]) < 5 && getState() != IntakeState::Idle) {
        if (jamStartTime == 0) {
          jamStartTime = pros::millis();
        }

        uint32_t jamTimer = pros::millis() - jamStartTime;

        if (armLoading) {
          if (jamTimer > 1000) {
            setState(IntakeState::Idle);
          }
        } else {
          if (jamTimer > 150) {
            setState(IntakeState::Jam);
          }
        }
      } else {
        jamStartTime = 0;
        jamTimer = 0;
      }
    }

    // ✂️ Sorter logic — DISABLED during SpinUntil
    if (getState() != IntakeState::SpinUntil) {
      if (teamColor == team::blue) {
        if (((color->get_hue() < 30) || (color->get_hue() > 300)) &&
            (color->get_proximity() > 200)) {
          sort_time = pros::millis();
        }
      } else if (teamColor == team::red) {
        if ((color->get_hue() > 215 && color->get_hue() < 290) &&
            (color->get_proximity() > 200)) {
          sort_time = pros::millis();
        }
      } else if (teamColor == team::skills) {
        sort_override;
      }

      if (pros::millis() - sort_time < 225 && !sort_override) {
        sort->extend();
      } else if (!sort_override) {
        sort->retract();
      }
    }

    // Intake state machine
    switch (getState()) {
      case IntakeState::Idle:
        motors->move(0);
        break;

      case IntakeState::In:
        motors->move(127);
        break;

      case IntakeState::Out:
        motors->move(-127);
        break;

      case IntakeState::Jam:
        motors->move(-127);
        if (armLoading) {
          setState(IntakeState::Idle);
          break;
        } else {
          pros::delay(250);
          setState(IntakeState::In);
          break;
        }
        setState(IntakeState::In);
        break;

      case IntakeState::SpinUntil:
        if (spinUntilStartTime == 0) {
          spinUntilStartTime = pros::millis();
        }

        motors->move(127 * 0.55);
        intake.setJamEnabled(false);

        bool ringDetected = false;

        if (teamColor == team::red) {
          if (((color->get_hue() < 30) || (color->get_hue() > 300)) &&
              ((color2.get_hue() < 30) || (color2.get_hue() > 300)) &&
              (color->get_proximity() > 200)) {
            ringDetected = true;
          }
        } else if (teamColor == team::blue) {
          if ((color->get_hue() > 180 && color->get_hue() < 330) &&
              (color2.get_hue() > 180 && color2.get_hue() < 330) &&
              (color->get_proximity() > 200)) {
            ringDetected = true;
          }
        }

        if (ringDetected || (pros::millis() - spinUntilStartTime >= 5000)) {
          spinUntilStartTime = 0;
          setState(IntakeState::Idle);
          intake.setJamEnabled(true);
        }
        break;
    }

    pros::delay(10);
  }
}
