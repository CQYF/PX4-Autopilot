#pragma once

#include <uORB/topics/vehicle_status.h>

#define WACM_MODE_STABILIZED	(vehicle_status_s::NAVIGATION_STATE_EXTERNAL1)
#define WACM_MODE_ACRO		(vehicle_status_s::NAVIGATION_STATE_EXTERNAL2)
#define WACM_MODE_MANUAL	(vehicle_status_s::NAVIGATION_STATE_EXTERNAL3)
#define WACM_MODE_AUTO_DIVE	(vehicle_status_s::NAVIGATION_STATE_EXTERNAL4)

