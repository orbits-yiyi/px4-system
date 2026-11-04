// Teaching example, using the PX4 external-module interface.
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <drivers/drv_hrt.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/learn_status.h>
#include <uORB/topics/parameter_update.h>

#include "PeriodPolicy.hpp"

class LearnStatus final : public ModuleBase<LearnStatus>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	LearnStatus() : ModuleParams(nullptr), ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::test1) {}

	bool init()
	{
		updateParams();
		_period_us = learning::period_us(_param_period.get());
		ScheduleOnInterval(_period_us);
		return true;
	}

	static int task_spawn(int argc, char *argv[])
	{
		LearnStatus *instance = new LearnStatus();

		if (instance) {
			_object.store(instance);
			_task_id = task_id_is_work_queue;

			if (instance->init()) {
				return PX4_OK;
			}
		}

		delete instance;
		_object.store(nullptr);
		_task_id = -1;
		return PX4_ERROR;
	}

	int print_status() override
	{
		// Mutable sample fields belong to the queue thread: read them via uORB.
		PX4_INFO("running; inspect: listener learn_status -n 1");
		return 0;
	}

	static int custom_command(int argc, char *argv[])
	{
		return print_usage("unknown command");
	}

	static int print_usage(const char *reason = nullptr)
	{
		if (reason) {
			PX4_WARN("%s", reason);
		}

		PRINT_MODULE_DESCRIPTION("Teaching telemetry on a work queue; no actuator output.");
		PRINT_MODULE_USAGE_NAME("learn_status", "example");
		PRINT_MODULE_USAGE_COMMAND("start");
		PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
		return 0;
	}

private:
	void Run() override
	{
		if (should_exit()) {
			ScheduleClear();
			exit_and_cleanup();
			return;
		}

		parameter_update_s update{};

		if (_parameter_update_sub.update(&update)) {
			updateParams();
			const uint32_t next_period = learning::period_us(_param_period.get());

			if (next_period != _period_us) {
				_period_us = next_period;
				ScheduleOnInterval(_period_us);
			}
		}

		learn_status_s data{};
		data.timestamp = hrt_absolute_time();
		data.sample_count = ++_sample_count;
		data.period_us = _period_us;
		data.requested_period_ms = _param_period.get();
		data.period_limited = data.requested_period_ms < 20 || data.requested_period_ms > 2000;
		_publication.publish(data);
	}

	uORB::Publication<learn_status_s> _publication{ORB_ID(learn_status)};
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
	uint32_t _sample_count{0};
	uint32_t _period_us{100000};
	DEFINE_PARAMETERS((ParamInt<px4::params::LEARN_PERIOD>) _param_period)
};

extern "C" __EXPORT int learn_status_main(int argc, char *argv[])
{
	return LearnStatus::main(argc, argv);
}
