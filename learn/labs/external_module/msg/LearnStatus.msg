# Teaching module telemetry. No flight-control or actuator semantics.
uint64 timestamp              # Publication time, microseconds since PX4 boot
uint32 sample_count           # Starts at 1 after module start; wraps at uint32 limit
uint32 period_us              # Effective requested scheduling interval, microseconds
int32 requested_period_ms     # Raw parameter value before defensive limits
bool period_limited           # True if the requested period was clamped
