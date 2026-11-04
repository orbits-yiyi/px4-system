#pragma once

#include <cstdint>

namespace learning
{
constexpr uint32_t period_us(int32_t requested_ms)
{
	return static_cast<uint32_t>(requested_ms < 20 ? 20 : requested_ms > 2000 ? 2000 : requested_ms) * 1000U;
}
} // namespace learning
