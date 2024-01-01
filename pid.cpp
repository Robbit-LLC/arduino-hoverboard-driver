#include <cmath>
#include <algorithm>
#include "pid.hpp"
#include <DebugLog.h>

void PID::init(double f, double p, double i, double d, double i_max, double i_min, bool antiwindup, double out_max,
		double out_min)
{
	f_ = f;

	setGains(p, i, d, i_max, i_min, antiwindup);

	reset();

	error_ = 0.0;

	out_min_ = out_min;
	out_max_ = out_max;

}

double PID::operator()(const double &measured_value, const double &setpoint, int64_t duration)
{
	// Compute error terms
	error_ = setpoint - measured_value;

	// Reset the i_error in case the p_error and the setpoint is zero
	// Otherwise there will always be a constant i_error_ that won't vanish
	if (0.0 == setpoint && 0.0 == error_)
	{
		reset();
	}

	// Use control_toolbox::Pid::computeCommand()
	double output = computeCommand(error_, duration);

	// Compute final output including feed forward term
	output = f_ * setpoint + output;
	//output = clamp(output, out_min_, out_max_);

	return output;
}

void PID::setParameters(double f, double p, double i, double d, double i_max, double i_min, bool antiwindup)
{
	f_ = f;
	setGains(p, i, d, i_max, i_min, antiwindup);
}

void PID::setOutputLimits(double output_max, double output_min)
{
	out_max_ = output_max;
	out_min_ = output_min;
}


double PID::clamp(const double &value, const double &lower_limit, const double &upper_limit)
{
	if (value > upper_limit)
	{
		return upper_limit;
	}
	else if (value < lower_limit)
	{
		return lower_limit;
	}

	return value;
}

void PID::reset()
{
	p_error_last_ = 0.0;
	p_error_ = 0.0;
	i_error_ = 0.0;
	d_error_ = 0.0;
	cmd_ = 0.0;
}

void PID::setGains(double p, double i, double d, double i_max, double i_min, bool antiwindup)
{
	Gains gains(p, i, d, i_max, i_min, antiwindup);

	setGains(gains);
}

void PID::setGains(const Gains &gains)
{
	if (gains.i_min_ > gains.i_max_)
	{
		LOG_INFO("received i_min > i_max, skip new gains");
	}
	else
	{
		gains_ = gains;
	}
}

double PID::computeCommand(double error, uint64_t dt)
{
	if (dt == 0 || std::isnan(error) || std::isinf(error))
	{
		return 0.0;
	}

	error_dot_ = d_error_;

	// Calculate the derivative error
	error_dot_ = (error - p_error_last_) / (dt / 1e9);
	p_error_last_ = error;

	return computeCommand(error, error_dot_, dt);
}

double PID::computeCommand(double error, double error_dot, uint64_t dt)
{
	double p_term;
	double d_term;
	double i_term;
	p_error_ = error;  // this is error = target - state
	d_error_ = error_dot;

	if (dt == 0 || std::isnan(error) || std::isinf(error) || std::isnan(error_dot) || std::isinf(error_dot))
	{
		return 0.0;
	}

	// Calculate proportional contribution to command
	p_term = gains_.p_gain_ * p_error_;

	// Calculate the integral of the position error
	i_error_ += (dt / 1e9) * p_error_;

	if (gains_.antiwindup_ && gains_.i_gain_ != 0)
	{
		// Prevent i_error_ from climbing higher than permitted by i_max_/i_min_
		auto [lowerBound, upperBound] =
				std::minmax<double>(gains_.i_min_ / gains_.i_gain_, gains_.i_max_ / gains_.i_gain_);
		i_error_ = std::clamp(i_error_, lowerBound, upperBound);
	}

	// Calculate integral contribution to command
	i_term = gains_.i_gain_ * i_error_;

	if (!gains_.antiwindup_)
	{
		// Limit i_term so that the limit is meaningful in the output
		i_term = std::clamp(i_term, gains_.i_min_, gains_.i_max_);
	}

	// Calculate derivative contribution to command
	d_term = gains_.d_gain_ * d_error_;

	// Compute the command
	cmd_ = p_term + i_term + d_term;

	return cmd_;
}

void PID::setCurrentCmd(double cmd)
{
	cmd_ = cmd;
}

double PID::getDerivativeError() const
{
	return error_dot_;
}

double PID::getCurrentCmd() const
{
	return cmd_;
}

void PID::getCurrentPIDErrors(double &pe, double &ie, double &de) const
{
	pe = p_error_;
	ie = i_error_;
	de = d_error_;
}
