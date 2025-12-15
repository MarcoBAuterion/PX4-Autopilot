/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file uwb_to_cartesian_main.cpp
 * @brief Simple module that converts raw UWB measurements (range + AoA) into
 *        cartesian relative & absolute landing target pose without filtering.
 *
 * This is a lightweight alternative to the vision_target_estimator position fusion.
 * No Kalman filtering is performed; each valid UWB sample is converted and published
 * directly as a landing_target_pose message.
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <drivers/drv_hrt.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/landing_target_pose.h>
#include <uORB/topics/sensor_uwb.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
#include <parameters/param.h>
#include <lib/parameters/param.h>

#include <lib/conversion/rotation.h>
#include <matrix/Quaternion.hpp>
#include <matrix/Vector3.hpp>
#include <mathlib/mathlib.h>

using matrix::Vector3f;
using matrix::Quaternionf;

namespace uwb_to_cartesian
{

static bool thread_should_exit = false; // Deamon exit flag
static bool thread_running = false;     // Deamon status flag
static int daemon_task = -1;            // Task handle

static constexpr uint32_t UPDATE_RATE_HZ = 50; // publish loop
static constexpr float MAX_AOA_DEG = 60.f;     // same bounds as vision target estimator

// uORB handles
uORB::Publication<landing_target_pose_s> _landing_target_pub{ORB_ID(landing_target_pose)};
uORB::SubscriptionData<sensor_uwb_s> _uwb_sub{ORB_ID(sensor_uwb)};
uORB::SubscriptionData<vehicle_attitude_s> _att_sub{ORB_ID(vehicle_attitude)};
uORB::SubscriptionData<vehicle_local_position_s> _vlp_sub{ORB_ID(vehicle_local_position)};

// Parameters
static param_t _p_enable{PARAM_INVALID};
static param_t _p_timeout_s{PARAM_INVALID};
static param_t _p_out_ned{PARAM_INVALID};

static int32_t _enable{1};           // runtime enable, default ON
static float   _timeout_s{3.0f};     // seconds without UWB -> rel_pos_valid false
static int32_t _out_ned{1};          // 0: output in local drone body frame (FRD), 1: NED

static hrt_abstime _last_uwb_update{0};

// Simple conversion replicating logic from vision_target_estimator/common.h
static bool uwb_to_ned(const sensor_uwb_s &uwb, const Quaternionf &vehicle_att, Vector3f &rel_pos_ned)
{
	if (!PX4_ISFINITE(uwb.distance) || uwb.distance <= 0.f) { return false; }

	// Reject angles outside valid bounds
	if (fabsf(uwb.aoa_azimuth_dev) > MAX_AOA_DEG || fabsf(uwb.aoa_elevation_dev) > MAX_AOA_DEG) { return false; }

	const float theta = math::radians(uwb.aoa_azimuth_dev);
	const float phi   = math::radians(uwb.aoa_elevation_dev);
	const float d     = uwb.distance;

	// Local sensor frame deltas (same sign convention as VTE)
	// const float delta_z = -d * cosf(phi) * cosf(theta);
	// const float delta_y =  d * cosf(phi) * sinf(theta);
	// const float delta_x = -d * sinf(phi);

	const float delta_z = d * cosf(phi) * cosf(theta);
	const float delta_y =  -d * cosf(phi) * sinf(theta);
	const float delta_x = d * sinf(phi);

	const Vector3f rel_sensor{uwb.offset_x + delta_x, uwb.offset_y + delta_y, uwb.offset_z + delta_z};

	Quaternionf sensor_rot = get_rot_quaternion(static_cast<Rotation>(uwb.orientation));
	// body_to_ned = vehicle attitude * sensor rotation into body
	Quaternionf body_to_ned = vehicle_att * sensor_rot;

	rel_pos_ned = body_to_ned.rotateVector(rel_sensor);
	return true;
}

static bool uwb_to_body(const sensor_uwb_s &uwb, Vector3f &rel_pos_body)
{
	if (!PX4_ISFINITE(uwb.distance) || uwb.distance <= 0.f) { return false; }

	if (fabsf(uwb.aoa_azimuth_dev) > MAX_AOA_DEG || fabsf(uwb.aoa_elevation_dev) > MAX_AOA_DEG) { return false; }

	const float theta = math::radians(uwb.aoa_azimuth_dev);
	const float phi   = math::radians(uwb.aoa_elevation_dev);
	const float d     = uwb.distance;

	// Z pointing down, not sure why we would want that
	//const float delta_z = -d * cosf(phi) * cosf(theta);
	//const float delta_y =  d * cosf(phi) * sinf(theta);
	//const float delta_x = -d * sinf(phi);

	// Out of Z
	const float delta_z = d * cosf(phi) * cosf(theta);
	const float delta_y =  -d * cosf(phi) * sinf(theta);
	const float delta_x = d * sinf(phi);

	const Vector3f rel_sensor{uwb.offset_x + delta_x, uwb.offset_y + delta_y, uwb.offset_z + delta_z};
	Quaternionf sensor_rot = get_rot_quaternion(static_cast<Rotation>(uwb.orientation));
	rel_pos_body = sensor_rot.rotateVector(rel_sensor);
	return true;
}

static void publish_pose(const sensor_uwb_s &uwb, const vehicle_attitude_s &att, const vehicle_local_position_s &vlp)
{
	Quaternionf q_att(&att.q[0]);
	Vector3f rel_pos;

	const bool use_ned = (_out_ned != 0);
	bool ok = false;

	if (use_ned) {
		ok = uwb_to_ned(uwb, q_att, rel_pos);

	} else {
		ok = uwb_to_body(uwb, rel_pos);
	}

	if (!ok) { return; }

	landing_target_pose_s msg{};
	msg.timestamp = hrt_absolute_time();

	// Relative position
	msg.rel_pos_valid = true;
	msg.x_rel = rel_pos(0);
	msg.y_rel = rel_pos(1);
	msg.z_rel = rel_pos(2);

	// No velocity estimate (static assumption)
	msg.is_static = true;
	msg.rel_vel_valid = false;
	msg.vx_rel = 0.f; msg.vy_rel = 0.f; msg.vz_rel = 0.f;

	// Crude variance based on range uncertainty (~2 cm fraction + small floor)
	const float pos_var = fmaxf(math::sq(uwb.distance * 0.02f) + 4e-4f, 1e-3f);
	msg.cov_x_rel = pos_var;
	msg.cov_y_rel = pos_var;
	msg.cov_z_rel = pos_var;
	msg.cov_vx_rel = NAN; msg.cov_vy_rel = NAN; msg.cov_vz_rel = NAN;

	// Absolute position only if local position valid and NED output selected
	if (use_ned && vlp.xy_valid && vlp.z_valid) {
		msg.abs_pos_valid = true;
		msg.x_abs = rel_pos(0) + vlp.x;
		msg.y_abs = rel_pos(1) + vlp.y;
		msg.z_abs = rel_pos(2) + vlp.z;

	} else {
		msg.abs_pos_valid = false;
		msg.x_abs = NAN; msg.y_abs = NAN; msg.z_abs = NAN;
	}

	_landing_target_pub.publish(msg);
}

static void loop()
{
	vehicle_attitude_s att{};
	vehicle_local_position_s vlp{};
	sensor_uwb_s uwb{};

	while (!thread_should_exit) {
		// parameters update (lightweight)
		if (_p_enable == PARAM_INVALID) {
			_p_enable = param_find("UWB2C_EN");
			_p_timeout_s = param_find("UWB2C_TOUT");
			_p_out_ned = param_find("UWB2C_OUT_NED");
		}

		if (_p_enable != PARAM_INVALID) { param_get(_p_enable, &_enable); }

		if (_p_timeout_s != PARAM_INVALID) { param_get(_p_timeout_s, &_timeout_s); }

		if (_p_out_ned != PARAM_INVALID) { param_get(_p_out_ned, &_out_ned); }

		// Update attitude & local position (if faster than UWB we still use latest)
		if (_att_sub.update()) {
			att = _att_sub.get();
		}

		if (_vlp_sub.update()) {
			vlp = _vlp_sub.get();
		}

		if (_enable) {
			if (_uwb_sub.update()) {
				uwb = _uwb_sub.get();
				_last_uwb_update = uwb.timestamp;
				publish_pose(uwb, att, vlp);

			} else {
				// If timed out, publish an invalid message to clear rel_pos_valid
				const float tout_us = _timeout_s * 1e6f;

				if (_last_uwb_update != 0 && (hrt_absolute_time() - _last_uwb_update) > (hrt_abstime)tout_us) {
					landing_target_pose_s msg{};
					msg.timestamp = hrt_absolute_time();
					msg.rel_pos_valid = false;
					msg.rel_vel_valid = false;
					msg.abs_pos_valid = false;
					msg.x_rel = NAN; msg.y_rel = NAN; msg.z_rel = NAN;
					msg.x_abs = NAN; msg.y_abs = NAN; msg.z_abs = NAN;
					_landing_target_pub.publish(msg);
					// Prevent spamming: reset last update so we only clear once until next sample
					_last_uwb_update = 0;
				}
			}
		}

		px4_usleep(1000000 / UPDATE_RATE_HZ);
	}
}

extern "C" __EXPORT int uwb_to_cartesian_main(int argc, char *argv[]);

int uwb_to_cartesian_main(int argc, char *argv[])
{
	if (argc < 2) {
		PX4_INFO("usage: uwb_to_cartesian {start|stop|status}");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {
		if (thread_running) {
			PX4_INFO("already running");
			return 0;
		}

		thread_should_exit = false;
		daemon_task = px4_task_spawn_cmd("uwb_to_cartesian",
						 SCHED_DEFAULT,
						 SCHED_PRIORITY_DEFAULT,
						 1600,
						 (px4_main_t)loop,
						 nullptr);

		if (daemon_task < 0) {
			PX4_ERR("task start failed");
			return -1;
		}

		thread_running = true;
		PX4_INFO("started");
		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;

		PX4_INFO("stopping");
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		PX4_INFO("%s", thread_running ? "running" : "not running");
		return 0;
	}

	PX4_INFO("unrecognized command");
	return 1;
}

} // namespace uwb_to_cartesian


// https://www.amazon.de/dp/B092J82XVR/ref=sspa_dk_detail_1?pd_rd_i=B0D72VTKS7&pd_rd_w=bMK16&content-id=amzn1.sym.bf6dbf94-e926-4351-8952-c09f45cdef70&pf_rd_p=bf6dbf94-e926-4351-8952-c09f45cdef70&pf_rd_r=52ZJP3F3KQMX432QZNJ3&pd_rd_wg=EAmAb&pd_rd_r=7c89c97e-4836-48b1-8c72-31ce8c7d90e2&aref=WvYDeQGe0T&sp_csd=d2lkZ2V0TmFtZT1zcF9kZXRhaWw&th=1
