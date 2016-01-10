/*
 * PID Controller Implementation in C++
 * 
 * Created by Joshua Saxby (aka @saxbophone) on 6 Jan, 2016
 * 
 * A hopefully minial implementation of the PID algorithm in C++.
 *
 * See LICENSE for licensing details.
 */

#include "pid.h"


// set class constants (default calibration values)
const double PidController::KP_DEFAULT = 1.0;
const double PidController::KI_DEFAULT = 1.0;
const double PidController::KD_DEFAULT = 1.0;

// set all state variables to 0.0
void PidController::state() {
    actual = 0.0;
    target = 0.0;
    output = 0.0;
    time_delta = 0.0;
    previous_error = 0.0;
    integral = 0.0;
}

// set all state variables to 0.0, except actual and target which we provide values for
void PidController::state(double _actual, double _target) {
    PidController::state();
    actual = _actual;
    target = _target;
}

// set calibration attributes to given values
void PidController::calibrate(double _kp, double _ki, double _kd) {
    kp = _kp;
    ki = _ki;
    kd = _kd;
}

// set calibration attributes to default values
void PidController::calibrate() {
    PidController::calibrate(KP_DEFAULT, KI_DEFAULT, KD_DEFAULT);
}

// default constructor
PidController() {
    PidController::calibrate();
    PidController::state();
}

// constructor with provided start values
PidController(double initial, double target) {
    PidController::calibrate();
    PidController::state(initial, target);
}

// constructor with provided calibration values
PidController(double kp, double ki, double kd) {
    PidController::calibrate(kp, ki, kd)
    PidController::state();
}

// constructor with provided calibration and start values
PidController(double kp, double ki, double kd, double initial, double target) {
    PidController::calibrate(kp, ki, kd);
    PidController::state(initial, target);
}

// reset state
void PidController::reset() {
    PidController::state();
}

// reset state and reset calibration attributes to defaults
void PidController::recalibrate() {
    PidController::calibrate();
    PidController::state();
}

// reset state and update values of calibration attributes
void PidController::recalibrate(double kp, double ki, double kd) {
    PidController::calibrate(kp, ki, kd);
    PidController::state();
}
