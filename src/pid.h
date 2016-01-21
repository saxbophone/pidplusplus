/*
 * PID Controller Implementation in C++
 * 
 * Created by Joshua Saxby (aka @saxbophone) on 6 Jan, 2016
 * 
 * A hopefully minial implementation of the PID algorithm in C++.
 *
 * See LICENSE for licensing details.
 */

// protection against multiple includes
#ifndef SAXBOPHONE_PID_H
#define SAXBOPHONE_PID_H


class PidController {
    private:
        // calibration values
        double kp; // Proportional gain
        double ki; // Integral gain
        double kd; // Derivative gain
        // calibration default constants
        static const double KP_DEFAULT;
        static const double KI_DEFAULT;
        static const double KD_DEFAULT;
        // actively changing values
        double actual; // The actual reading as measured
        double target; // The desired reading
        // the modified output value calculated by the algorithm
        double output;
        // Time since last update
        double time_delta;
        // previously calculated error between actual and target
        double previous_error;
        double integral; // Sum of integral error over time
        // methods
        // initialises state at blank starting point
        void state();
        // initialises state to given values
        void state(double initial, double target);
        // sets values of calibration attributes 
        void calibrate(double kp, double ki, double kd);
        // sets values of calibration attributes to defaults
        void calibrate();
        // calculate the PID output and store in output attribute
        void calculate();
    public:
        // constructors
        PidController(); // default constructor
        // constructor with provided start values
        PidController(double initial, double target);
        // constructor with provided calibration values
        PidController(double kp, double ki, double kd);
        // constructor with provided calibration and start values
        PidController(double kp, double ki, double kd, double initial, double target);
        // methods
        void reset(); // resets state
        // resets state and resets calibration attributes to defaults
        void recalibrate();
        // resets state and updates values of calibration attributes
        void recalibrate(double kp, double ki, double kd);
        // retrieves the current output value (no calculation performed)
        double get_output();
        // calculates the PID output and returns it
        double update();
        // given the actual value, calculate PID output and return it
        double update(double actual);
        // given the target value and the actual value, calculate PID output and return it
        double update(double actual, double target);
};


// end of header
#endif
