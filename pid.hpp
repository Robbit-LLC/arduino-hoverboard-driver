#pragma once

class PID
{
public:
	struct Gains
	{
		// Optional constructor for passing in values without antiwindup
		Gains(double p, double i, double d, double i_max, double i_min) : p_gain_(p), i_gain_(i), d_gain_(d),
																		  i_max_(i_max), i_min_(i_min)
		{
		}

		// Optional constructor for passing in values
		Gains(double p, double i, double d, double i_max, double i_min, bool antiwindup) : p_gain_(p), i_gain_(i),
																						   d_gain_(d), i_max_(i_max),
																						   i_min_(i_min),
																						   antiwindup_(antiwindup)
		{
		}

		// Default constructor
		Gains() : p_gain_(0.0), i_gain_(0.0), d_gain_(0.0), i_max_(0.0), i_min_(0.0)
		{
		}

		double p_gain_;   /**< Proportional gain. */
		double i_gain_;   /**< Integral gain. */
		double d_gain_;   /**< Derivative gain. */
		double i_max_;    /**< Maximum allowable integral term. */
		double i_min_;    /**< Minimum allowable integral term. */
		bool antiwindup_{ false }; /**< Antiwindup. */
	};

	PID() = default;

	~PID() = default;

	/**
  * @brief Initialize the
  *
  * @param kF The feed forward gain.
  * @param kP The proportional gain.
  * @param kI The integral gain.
  * @param kD The derivative gain.
  * @param i_max The max integral windup.
  * @param i_min The min integral windup.
  * @param antiwindup
  * @param out_min The min computed output.
  * @param out_max The max computed output.
  */
	void init(double f, double p, double i, double d, double i_max, double i_min, bool antiwindup, double out_max,
			double out_min);

	/**
	 * @brief Compute PID output value from error using process value, set point and time period
	 *
	 * @param measured_value The process value measured by sensors.
	 * @param setpoint The desired target value.
	 * @param dt The delta time or period since the last call.
	 * @return double Computed PID output value.
	 */
	double operator()(const double &measured_value, const double &setpoint, int64_t duration);

	/**
	 * @brief Set the Parameters using the Gains object of control_toolbox::Pid
	 *
	 * @param f The feed forward gain.
	 * @param p The proportional gain.
	 * @param i The integral gain.
	 * @param d The derivative gain.
	 * @param i_max The max integral windup.
	 * @param i_min The min integral windup.
	 * @param antiwindup Enable or disable antiwindup check.
	 */
	void setParameters(double f, double p, double i, double d, double i_max, double i_min, bool antiwindup = false);

	/**
	 * @brief Set the Output Limits of the PID controller
	 *
	 * @param out_min
	 * @param out_max
	 */
	void setOutputLimits(double out_min, double out_max);

	/**
	 * @brief Clam given value to upper and lower limits.
	 *
	 * @param value Input value that's possibly clamped.
	 * @param lower_limit Lower limit which the value must not exceed.
	 * @param upper_limit Upper limit which the value must not exceed.
	 * @return double Clamped value to range in between [lower_limit, upper_limit].
	 */
	double clamp(const double &value, const double &lower_limit, const double &upper_limit);

	/**
	 * @brief Get the current error.
	 *
	 * @return double The current error computed from the measured and target value.
	 */
	[[nodiscard]] inline double getError() const
	{
		return error_;
	};

	/*!
 * \brief Reset the state of this PID controller
 */
	void reset();

/*!
 * \brief Set PID gains for the controller.
 * \param p The proportional gain.
 * \param i The integral gain.
 * \param d The derivative gain.
 * \param i_max The max integral windup.
 * \param i_min The min integral windup.
 *
 * \note New gains are not applied if i_min > i_max
 */
	void setGains(double p, double i, double d, double i_max, double i_min, bool antiwindup = false);

/*!
 * \brief Set PID gains for the controller.
 * \param gains A struct of the PID gain values
 *
 * \note New gains are not applied if gains.i_min_ > gains.i_max_
 */
	void setGains(const Gains &gains);

/*!
 * \brief Set the PID error and compute the PID command with nonuniform time
 * step size. The derivative error is computed from the change in the error
 * and the timestep \c dt.
 *
 * \param error  Error since last call (error = target - state)
 * \param dt Change in time since last call in nanoseconds
 *
 * \returns PID command
 */
	double computeCommand(double error, uint64_t dt);

/*!
 * \brief Set the PID error and compute the PID command with nonuniform
 * time step size. This also allows the user to pass in a precomputed
 * derivative error.
 *
 * \param error Error since last call (error = target - state)
 * \param error_dot d(Error)/dt since last call
 * \param dt Change in time since last call in nanoseconds
 *
 * \returns PID command
 */
	double computeCommand(double error, double error_dot, uint64_t dt);

/*!
 * \brief Set current command for this PID controller
 */
	void setCurrentCmd(double cmd);

/*!
 * \brief Return current command for this PID controller
 */
	[[nodiscard]] double getCurrentCmd() const;

/*!
 * \brief Return derivative error
 */
	[[nodiscard]] double getDerivativeError() const;

/*!
 * \brief Return PID error terms for the controller.
 * \param pe  The proportional error.
 * \param ie  The integral error.
 * \param de  The derivative error.
 */
	void getCurrentPIDErrors(double &pe, double &ie, double &de) const;

private:
	double f_{ 0.0 };
	double error_{ 0.0 };
	double out_min_{};
	double out_max_{};

	Gains gains_{};

	double p_error_last_{}; /**< _Save position state for derivative state calculation. */
	double p_error_{};      /**< Position error. */
	double i_error_{};      /**< Integral of position error. */
	double d_error_{};      /**< Derivative of position error. */
	double cmd_{};          /**< Command to send. */
	double error_dot_{};    /**< Derivative error */
};
