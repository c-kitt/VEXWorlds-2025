// lift.cpp
#include "lib/lift.hpp"
#include "pros/abstract_motor.hpp"
#include "robotconfig.h"
#include <cstdint>
#include <cmath>
#include <iostream>

using namespace lib;

void Lift::loop() {
  uint32_t now = pros::millis();
  //liftSensor.reset_position();
  motors->tare_position();
  motors->set_brake_mode_all(pros::MotorBrake::hold);

  while (true) {
    switch (getState()) {
      case LiftState::Stored:
        armLoading = false;
        if (target != REST_ANGLE) {
          liftpid.variables_reset();
          target = REST_ANGLE;
        }
        break;

      case LiftState::Recieve:
        armLoading = true;
        if (target != LOAD_ANGLE) {
          liftpid.variables_reset();
          target = LOAD_ANGLE;
        }
        break;

      case LiftState::Score:
        if (target != SCORE_ANGLE) {
          liftpid.variables_reset();
          target = SCORE_ANGLE;
        }
        break;

      case LiftState::Mid:
        if (target != MID_ANGLE) {
          liftpid.variables_reset();
          target = MID_ANGLE;
        }
        break;

      case LiftState::Alliance:
        if (target != ALLIANCE_ANGLE) {
          liftpid.variables_reset();
          target = ALLIANCE_ANGLE;
        }
        break;

      case LiftState::Bar:
        if (target != BAR_ANGLE) {
          liftpid.variables_reset();
          target = BAR_ANGLE;
        }
        break;

      case LiftState::AllianceMid:
        if (target != ALLIANCE_ANGLE) {
          liftpid.variables_reset();
          target = ALLIANCE_ANGLE;
        }
        break;

      case LiftState::GoalRush:
        if (target != RUSH_ANGLE) {
          liftpid.variables_reset();
          target = RUSH_ANGLE;
        }
        break;

      case LiftState::AllianceScore:
        if (target != ALLIANCE_SCORE_ANGLE) {
          liftpid.variables_reset();
          target = ALLIANCE_SCORE_ANGLE;
        }
        break;

      case LiftState::Tipping:
        if (target != TIPPING_ANGLE) {
          liftpid.variables_reset();
          target = TIPPING_ANGLE;
        }
        break;
    }

    liftpid.target_set(target / gearRatio);

    // Compute current error and PID output
    double currentPos = liftSensor.get_position() / 100.0;
    double desiredPos = target / gearRatio;
    double error      = desiredPos - currentPos;
    double output     = liftpid.compute_error(error, currentPos);

    // hold if within ±HOLD_TOLERANCE
    if (std::fabs(error) < HOLD_TOLERANCE) {
      motors->move(0);
    }
    else if (target == BAR_ANGLE) {
      motors->move(output * 0.5);
    } else if (target == ALLIANCE_ANGLE) {
      motors->move(output * 0.6);
    }
    else {
      motors->move(output);
    }

    pros::Task::delay_until(&now, 15);
  }
}

void Lift::itterateState(bool delta) {
  if (getState() == LiftState::Stored) {
    setState(LiftState::Recieve);
  } else if (getState() == LiftState::Recieve) {
    setState(LiftState::Score);
  } else if (getState() == LiftState::Score) {
    setState(LiftState::Stored);
  } else if (getState() == LiftState::Mid) {
    setState(LiftState::Score);
  } else if (getState() == LiftState::Alliance) {
    setState(LiftState::Score);
    isAlliance = !isAlliance;
  } else if (getState() == LiftState::Bar) {
    setState(LiftState::Stored);
  } else if (getState() == LiftState::AllianceMid) {
    setState(LiftState::Stored);
  } else if (getState() == LiftState::GoalRush) {
    setState(LiftState::Stored);
  } else if (getState() == LiftState::PreGoalRush) {
    setState(LiftState::Stored);
  } else if (getState() == LiftState::Tipping) {
    setState(LiftState::Stored);
  }
}

float Lift::getAngle() {
  return target;
}
