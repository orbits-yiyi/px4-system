px4_add_module(
	MODULE examples__learn_status
	MAIN learn_status
	SRCS LearnStatus.cpp
	EXTERNAL
)
# External modules are declared before platform libraries in PX4's top-level
# CMakeLists. Defer resolving this library until CMake's generation phase.
target_link_libraries(examples__learn_status PRIVATE px4_work_queue)
