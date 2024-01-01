#include "hoverboard.hpp"

#include <DebugLog.h>

Hoverboard::Hoverboard(HardwareSerial &hoverboardSerial)
{
	// Convert m/s to rad/s
	max_velocity /= wheel_radius;

	// Init PID controller
	pids[0].init(1.0, 0.0, 0.0, 0.0, 1.5, -1.5, true, max_velocity, -max_velocity);
	pids[0].setOutputLimits(-max_velocity, max_velocity);
	pids[1].init(1.0, 0.0, 0.0, 0.0, 1.5, -1.5, true, max_velocity, -max_velocity);
	pids[1].setOutputLimits(-max_velocity, max_velocity);

	// Store uart (serial instance)
	_hoverboardSerial = &hoverboardSerial;

	_uartConnected = true;
}

Hoverboard::~Hoverboard()
{
	if (_uartConnected)
	{
		_hoverboardSerial->end();
		_uartConnected = false;
	}
}

std::optional<std::pair<int16_t, int16_t>> Hoverboard::readEncoders()
{
	if (_uartConnected)
	{
		int i = 0;
		int r = 0;

		// available() returns number of characters available for reading from serial port
		while ((r = _hoverboardSerial->available()) > 0 && i++ < 1024)
		{
			auto byte = _hoverboardSerial->read();
			start_frame = ((uint16_t)(byte) << 8) | (uint8_t)prev_byte;

			// Read the start frame
			if (start_frame == START_FRAME)
			{
				p = (char *)&msg;
				*p++ = prev_byte;
				*p++ = byte;
				msg_len = 2;
			}
			else if (msg_len >= 2 && msg_len < sizeof(SerialFeedback))
			{
				// Otherwise, just read the message content until the end
				*p++ = byte;
				msg_len++;
			}

			if (msg_len == sizeof(SerialFeedback))
			{
				auto checksum = (uint16_t)(msg.start ^ msg.cmd1 ^ msg.cmd2 ^ msg.speedR_meas ^ msg.speedL_meas ^
										   msg.wheelR_cnt ^ msg.wheelL_cnt ^ msg.batVoltage ^ msg.boardTemp ^
										   msg.cmdLed);

				if (msg.start == START_FRAME && msg.checksum == checksum)
				{
					// FIXME: We could here potentially handle battery voltage and board temperature values,
					// but they are not needed at this moment (this method is fetching just encoder ticks, but it could be
					// refactored to be 'general' read function,
					// and having separate getter for each value - or some other mechanism).

					// NOTE: We will do nothing here, because we are interested just in cumulative pulse counts value at the end of reading (out of while loop)
				}
				else
				{
					LOG_WARN("Hoverboard checksum mismatch: %d vs %d", msg.checksum, checksum);
				}
				msg_len = 0;
			}
			prev_byte = byte;
		}

		if (i > 0)
			_lastReadSeconds = millis() / 1000;

		if (r < 0 && errno != EAGAIN)
			LOG_ERROR("Reading from provided uart failed: %d", r);
	}

	if (((millis() / 1000) - _lastReadSeconds) > 1)
	{
		LOG_ERROR("Timeout reading from provided uart failed");

	}

	// Get difference in cnt (considering overflow or underflow)
	int16_t leftWheelCntDiff = handleWraparound(msg.wheelL_cnt, last_wheelcountL);
	int16_t rightWheelCntDiff = handleWraparound(msg.wheelR_cnt, last_wheelcountR);

	// Update last counts
	last_wheelcountL = msg.wheelL_cnt;
	last_wheelcountR = msg.wheelR_cnt;

	return std::make_pair(leftWheelCntDiff, rightWheelCntDiff);
}

void Hoverboard::write(float measuredLeftWheelVel, float measuredRightWheelVel, float leftWheelSetpoint,
		float rightWheelSetpoint,
		int64_t period)
{
	if (!_uartConnected)
	{
		return;
	}

	std::array<double, 2> pid_outputs{};
	pid_outputs[0] = pids[0](measuredLeftWheelVel, leftWheelSetpoint, period);
	pid_outputs[1] = pids[1](measuredRightWheelVel, rightWheelSetpoint, period);

	std::array<double, 2> set_speed = { pid_outputs[0], pid_outputs[1] };

	// Calculate steering from difference of a left and right
	const double speed = (set_speed[0] + set_speed[1]) / 2.0;
	const double steer = (set_speed[0] - speed) * 2.0;

	SerialCommand command;
	command.start = (uint16_t)START_FRAME;
	command.steer = (int16_t)(steer * _steerCoeff);
	command.speed = (int16_t)(speed * _speedCoeff);
	command.checksum = (uint16_t)(command.start ^ command.steer ^ command.speed);

	int rc = _hoverboardSerial->write(reinterpret_cast<const uint8_t *>(&command), sizeof(command));
	if (rc < 0)
	{
		LOG_ERROR("Error writing to hoverboard serial port");
	}
}

void Hoverboard::setSpeedLimitCoeffs(float speedCoeff, float steerCoeff)
{
	_speedCoeff = speedCoeff;
	_steerCoeff = steerCoeff;
}
