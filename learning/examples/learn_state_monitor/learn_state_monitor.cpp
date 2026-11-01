/* SPDX-License-Identifier: BSD-3-Clause */

#include <px4_platform_common/module.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/defines.h>
#include <drivers/drv_hrt.h>
#include <parameters/param.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>
#include <errno.h>
#include <math.h>
#include <new>

extern "C" __EXPORT int learn_state_monitor_main(int argc, char *argv[]);

class LearnStateMonitor final : public ModuleBase<LearnStateMonitor>
{
public:
	static int task_spawn(int argc, char *argv[])
	{
		_task_id = px4_task_spawn_cmd("learn_state_monitor", SCHED_DEFAULT,
					    SCHED_PRIORITY_DEFAULT, 2400,
					    (px4_main_t)&run_trampoline, (char *const *)argv);

		if (_task_id < 0) {
			_task_id = -1;
			return -errno;
		}

		return 0;
	}

	static LearnStateMonitor *instantiate(int argc, char *argv[])
	{
		return new (std::nothrow) LearnStateMonitor();
	}

	static int custom_command(int argc, char *argv[]) { return print_usage("unknown command"); }

	static int print_usage(const char *reason = nullptr)
	{
		if (reason) { PX4_WARN("%s", reason); }

		PRINT_MODULE_USAGE_NAME("learn_state_monitor", "example");
		PRINT_MODULE_USAGE_COMMAND("start");
		PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
		return 0;
	}

	int print_status() override
	{
		PX4_INFO("Running; output rate is configured with LEARN_MON_HZ");
		return 0;
	}

	void run() override
	{
		uORB::Subscription position_sub{ORB_ID(vehicle_local_position)};
		uORB::Subscription status_sub{ORB_ID(vehicle_status)};
		uORB::Subscription land_sub{ORB_ID(vehicle_land_detected)};
		uORB::Subscription parameter_sub{ORB_ID(parameter_update)};
		vehicle_local_position_s position{};
		vehicle_status_s status{};
		vehicle_land_detected_s land{};
		const param_t rate_handle = param_find("LEARN_MON_HZ");
		hrt_abstime interval_us = 1000000;
		hrt_abstime last_print = 0;
		bool force_parameters = true;

		while (!should_exit()) {
			position_sub.update(&position);
			status_sub.update(&status);
			land_sub.update(&land);
			parameter_update_s update{};
			const bool parameters_changed = parameter_sub.update(&update);

			if (force_parameters || parameters_changed) {
				float rate_hz = 1.0f;

				if (rate_handle != PARAM_INVALID && param_get(rate_handle, &rate_hz) == 0
				    && PX4_ISFINITE(rate_hz) && rate_hz >= 0.2f && rate_hz <= 5.0f) {
					interval_us = static_cast<hrt_abstime>(1000000.0f / rate_hz);
				}

				force_parameters = false;
			}

			const hrt_abstime now = hrt_absolute_time();

			if (now - last_print >= interval_us) {
				const bool fresh = position.timestamp != 0 && now >= position.timestamp
						   && now - position.timestamp < 500000;
				const bool armed = status.arming_state == vehicle_status_s::ARMING_STATE_ARMED;
				PX4_INFO("armed=%d landed=%d pos_seen=%d status_seen=%d land_seen=%d fresh=%d z_valid=%d z=%.2f vz=%.2f",
					 (int)armed, (int)land.landed, (int)(position.timestamp != 0),
					 (int)(status.timestamp != 0), (int)(land.timestamp != 0), (int)fresh,
					 (int)position.z_valid, (double)position.z, (double)position.vz);
				last_print = now;
			}

			px4_usleep(100000);
		}
	}
};

int learn_state_monitor_main(int argc, char *argv[])
{
	return LearnStateMonitor::main(argc, argv);
}
