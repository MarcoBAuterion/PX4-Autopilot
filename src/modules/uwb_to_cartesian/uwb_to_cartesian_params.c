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
 * @file uwb_to_cartesian_params.c
 * Parameters for the UWB to cartesian landing target publisher.
 */

#include <parameters/param.h>

/**
 * UWB-to-Cartesian enable
 *
 * Enable/disable publishing of landing_target_pose from UWB at runtime.
 *
 * @boolean
 * @group UWB to Cartesian
 */
PARAM_DEFINE_INT32(UWB2C_EN, 1);

/**
 * UWB timeout
 *
 * Time without UWB updates after which rel_pos_valid is cleared.
 *
 * @unit s
 * @min 0.0
 * @max 50.0
 * @decimal 1
 * @group UWB to Cartesian
 */
PARAM_DEFINE_FLOAT(UWB2C_TOUT, 3.0f);

/**
 * Output frame selection
 *
 * 0: Output relative position in the drone's local body frame (FRD)
 * 1: Output relative position in NED (vehicle-carried)
 *
 * @boolean
 * @group UWB to Cartesian
 */
PARAM_DEFINE_INT32(UWB2C_OUT_NED, 1);
