#include "pid_controller.h"

Pid_controller::Pid_controller()
{
    kp = ki = kd = feed_forward = max_output = 0;
    first_time = true;
}

Pid_controller::Pid_controller(double kp, double ki, double kd, double feed_forward, double max_output, double max_i, double updateInterval)
{
    this->max_i = max_i;
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    this->feed_forward = feed_forward;
    this->max_output = max_output;
    this->T = updateInterval;
    first_time = true;
}

void Pid_controller::set_parameters(double kp, double ki, double kd, double feed_forward, double max_output, double max_i, double updateInterval)
{
    this->max_i = max_i;
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    this->feed_forward = feed_forward;
    this->max_output = max_output;
    this->T = updateInterval;
}

void Pid_controller::reset()
{
    this->error_prev = 0.0;
    this->first_time = true;
}

double Pid_controller::update(double error)
{
    this->error = error;
    // proportional
    p = kp * error;
    ROS_INFO("T = %f", T);
    // integral
    i += ki * error * T;

    if (i > max_i) {
        i = max_i;
    } else if (i < -max_i) {
        i = -max_i;
    }

    // derivative
    if (first_time) {
        d = 0;
        first_time = false;
    } else {
        d = kd * (error - error_prev) / T;
    }

    error_prev = error; // save for next iteration

    // calculate feed forward to avoid deadband without movement
    if (error < 0) {
        ff = -feed_forward;
    } else {
        ff = feed_forward;
    }

    // calc. output
    output = ff + p + i + d;

    // limit to max output
    if (output > max_output) {
        output = max_output;
    } else if (output < -max_output) {
        output = -max_output;
    }

    return output;
}

std::vector<double> Pid_controller::getLatestUpdateValues()
{
    std::vector<double> outVec = { error, output, p, i, d, ff };
    return outVec;
}
