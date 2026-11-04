#include "reference_limiter.hpp"
#include "../external_module/src/learn_status/PeriodPolicy.hpp"
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <limits>

static int checks = 0;

static void require(bool condition, const char *description)
{
	++checks;
	if (!condition) {
		std::cerr << "FAIL: " << description << '\n';
		std::exit(1);
	}
}

int main()
{
	using learning::limit_reference;
	require(learning::period_us(INT32_MIN) == 20000, "negative period never becomes a huge unsigned delay");
	require(learning::period_us(19) == 20000, "lower bound");
	require(learning::period_us(20) == 20000, "inclusive minimum");
	require(learning::period_us(100) == 100000, "milliseconds to microseconds");
	require(learning::period_us(2000) == 2000000, "inclusive maximum");
	require(learning::period_us(INT32_MAX) == 2000000, "clamp before multiplication");

	const auto forward = limit_reference(0, 10, 2, .25);
	require(forward.valid && forward.value == .5, "forward speed bound");
	const auto reverse = limit_reference(1, -10, 2, .25);
	require(reverse.valid && reverse.value == .5, "negative direction");
	require(limit_reference(1, 1.1, 2, .25).value == 1.1, "do not overshoot");
	require(limit_reference(1, 1, 2, .25).value == 1, "already at target");
	require(limit_reference(1, 10, 0, .25).value == 1, "zero speed holds reference");
	require(!limit_reference(1, 10, -1, .25).valid, "negative speed rejected");
	require(!limit_reference(1, 10, 1, 0).valid, "zero dt rejected");
	require(!limit_reference(1, 10, 1, -1).valid, "negative dt rejected");
	require(!limit_reference(1, NAN, 1, 1).valid, "NaN target rejected");
	require(!limit_reference(INFINITY, 10, 1, 1).valid, "non-finite state rejected");
	require(!limit_reference(1, 10, INFINITY, 1).valid, "non-finite speed rejected");
	require(!limit_reference(1, 10, 1, NAN).valid, "non-finite time rejected");
	require(!limit_reference(1, 10, std::numeric_limits<double>::max(), 2).valid, "step overflow rejected");

	double reference = 0;
	for (int i = 0; i < 20; ++i) {
		const auto next = limit_reference(reference, 1, .7, .1);
		require(next.valid && next.value >= reference && next.value <= 1, "trajectory monotonic and bounded");
		require(next.value - reference <= .070000000001, "each trajectory step obeys speed limit");
		reference = next.value;
	}
	require(reference == 1, "trajectory reaches target exactly");
	std::cout << "PASS: " << checks << " checks\n";
}
