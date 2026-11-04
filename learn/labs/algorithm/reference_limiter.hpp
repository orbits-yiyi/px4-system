#pragma once

#include <algorithm>
#include <cmath>

namespace learning
{
struct ReferenceStep {
	double value;
	bool valid;
};

// Stateless, one-dimensional slew limiter. Caller owns the current reference.
// Invalid input is rejected; value is the unchanged current reference.
inline ReferenceStep limit_reference(double current, double target, double max_speed, double dt)
{
	if (!std::isfinite(current) || !std::isfinite(target) || !std::isfinite(max_speed)
	    || !std::isfinite(dt) || max_speed < 0.0 || dt <= 0.0) {
		return {current, false};
	}

	const double limit = max_speed * dt;
	const double delta = target - current;

	if (!std::isfinite(limit) || !std::isfinite(delta)) {
		return {current, false};
	}

	return {current + std::max(-limit, std::min(delta, limit)), true};
}
} // namespace learning
