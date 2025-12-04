px4_add_module(
	MODULE modules__uwb_to_cartesian
	MAIN uwb_to_cartesian
	SRCS
		uwb_to_cartesian_main.cpp
	DEPENDS
)
