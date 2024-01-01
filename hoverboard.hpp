//
// Created by ognjen on 5/31/23.
//

#include <string>
#include "protocol.hpp"
#include "pid.hpp"
#include <optional>
#include <array>
#include <HardwareSerial.h>

class Hoverboard
{
public:
	explicit Hoverboard(HardwareSerial &hoverboardSerial);

	~Hoverboard();

	std::optional<std::pair<int16_t, int16_t>> readEncoders();

	void setSpeedLimitCoeffs(float speedCoeff = 0.02f, float steerCoeff = 0.02f);

	// Values are in RPM (Rotations per minute)
	void
	write(float measuredLeftWheelVel, float measuredRightWheelVel, float leftWheelSetpoint, float rightWheelSetpoint,
			int64_t period);

private:
	double wheel_radius{};
	double max_velocity = 1.0;

	// Hoverboard protocol
	int msg_len = 0;
	char prev_byte = 0;
	uint16_t start_frame = 0;
	char *p{};
	SerialFeedback msg{};

	std::array<PID, 2> pids;

	bool _uartConnected{ false };

	int64_t _lastReadSeconds{};

	// Last known encoder values
	int16_t last_wheelcountR{ 0 };
	int16_t last_wheelcountL{ 0 };

	// By default, it is set to 'full' speed capacity
	float _speedCoeff{ 1.f };
	// By default, it is set to 'full' steer capacity
	float _steerCoeff{ 1.f };

	HardwareSerial *_hoverboardSerial;
};

// Function to handle wraparound for encoder counts
constexpr int16_t handleWraparound(int16_t currentCount, int16_t lastCount)
{
	if (currentCount < lastCount - 4500)
	{
		// If the current count is significantly less than the last count, it wrapped around to the maximum value
		return static_cast<int16_t>(currentCount + 9000 - lastCount);
	}
	else if (currentCount - lastCount > 4500)
	{
		// If the current count is significantly greater than the last count, it wrapped around to the minimum value
		return static_cast<int16_t>(currentCount - 9000 - lastCount);
	}
	else
	{
		// Normal case without wraparound
		return static_cast<int16_t>(currentCount - lastCount);
	}
}
