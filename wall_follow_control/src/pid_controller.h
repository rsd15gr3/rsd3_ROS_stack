#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include<vector>
#include<ros/assert.h>
class Pid_controller
{
public:
    Pid_controller();
    Pid_controller(double kp, double ki, double kd, double feed_forward, double max_output, double max_i, double updateInterval);
	~Pid_controller();
    void set_parameters(double kp, double ki, double kd, double feed_forward, double max_output, double max_i, double updateInterval);
	void reset();
    double update(double error);
	std::vector<double> getLatestUpdateValues();
private:
    double T;
	double error, error_prev;
	double kp, ki, kd, feed_forward, max_output;
    double output;
	double p, i, d, ff;
    double T_prev;
    double max_i;
	bool first_time;
	// methods

};

#endif // PID_CONTROLLER_H
