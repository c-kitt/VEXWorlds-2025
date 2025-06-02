#include "pros/optical.hpp"
#pragma once

namespace lib {



class Color {

private:
    pros::Optical sensor = pros::Optical(20);

public:
    Color() {
        sensor.set_integration_time(5);
        sensor.set_led_pwm(100);
    }
    bool seesRed() {
        return (sensor.get_hue() < 30 || sensor.get_hue() > 300) && sensor.get_proximity() > 200;
    }
    bool seesBlue() {
        return (sensor.get_hue() > 180 && sensor.get_hue() < 330) && sensor.get_proximity() > 200;
    }
};
};