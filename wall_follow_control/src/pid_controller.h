#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include<vector>

class Pid_controller
{
public:
    Pid_controller();
    Pid_controller(double kp, double ki, double kd, double feed_forward, double max_output, double max_i);
	~Pid_controller();
    void set_parameters(double kp, double ki, double kd, double feed_forward, double max_output, double max_i);
	void reset();
    double update(double error, double T_now);
	std::vector<double> getLatestUpdateValues();
private:
	double updateInterval;
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
