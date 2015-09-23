#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include<vector>

class Pid_controller
{
public:
	Pid_controller(double timeInterval);
    Pid_controller(double timeInterval, double kp, double ki, double kd, double feed_forward, double max_output);
	~Pid_controller();
	void set_parameters(double kp, double ki, double kd, double feed_forward, double max_output);
	void reset();
	double update(double error);
	std::vector<double> getLatestUpdateValues();
private:
	double updateInterval;
	double error, error_prev;
	double kp, ki, kd, feed_forward, max_output;
	double integral, output;
	double p, i, d, ff;
	double T;
	bool first_time;
	// methods

};

#endif // PID_CONTROLLER_H
