// lift.hpp
#pragma once
#include "StateMachine.hpp"
#include "lib/TaskWrapper.hpp"
#include "liftPID.h"
#include "pros/adi.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.h"  // Include for intake motor
#include <memory>
#include <cmath>          // for std::fabs

namespace lib {

enum class LiftState {
  Stored, Recieve, Score, Mid, Alliance,
  AllianceScore, AllianceMid, Bar,
  GoalRush, PreGoalRush, Tipping
};

// PID constant struct
struct liftPIDConstants {
    double kP;
    double kI;
    double kD;
};

class Lift : public StateMachine<LiftState, LiftState::Stored>,
             public ryan::TaskWrapper {
private:
    std::shared_ptr<pros::MotorGroup> motors;
    pros::Motor& intakeMotor;
    const float REST_ANGLE           = 45;
    const float LOAD_ANGLE           = 20;
    const float SCORE_ANGLE          = -152;
    const float MID_ANGLE            = -50;
    const float ALLIANCE_SCORE_ANGLE = -190;
    const float ALLIANCE_ANGLE       = -117;
    const float BAR_ANGLE            = -105;
    const float RUSH_ANGLE           = -130;
    const float TIPPING_ANGLE        = -190;

    const float gearRatio;
    const liftPIDConstants constants;
    double target;
    allPID liftpid = allPID(constants.kP, constants.kI, constants.kD);

    static constexpr double HOLD_TOLERANCE = 7.0;  // dead-band for holding

public:
    Lift(pros::MotorGroup *motors,
         pros::Motor &intakeMotor,
         double gearRatio,
         liftPIDConstants constants)
      : motors(motors),
        intakeMotor(intakeMotor),
        gearRatio(gearRatio),
        constants(constants),
        liftpid(constants.kP, constants.kI, constants.kD) 
    {
        motors->set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
        motors->set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        target = REST_ANGLE;
        motors->tare_position(); // Initialize motor positions
    }

    void loop() override;
    float getAngle();
    void itterateState(bool delta);
    void setTarget(float target);
    void setVoltage(int voltage);

private:
    void handleStateActions();
    bool sensorResetDone = false;
};

} // namespace lib
