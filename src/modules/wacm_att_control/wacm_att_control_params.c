/****************************************************************************
 *
 *   Copyright (c) 2013-2023 PX4 Development Team. All rights reserved.
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
 * AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
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
 * @file wacm_att_control_params.c
 *
 * Parameters defined by the fixed-wing attitude control task
 *
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Thomas Gubler <thomas@px4.io>
 */

/**
 * Attitude Roll Time Constant
 *
 * This defines the latency between a roll step input and the achieved setpoint
 * (inverse to a P gain). Smaller systems may require smaller values.
 *
 * @unit s
 * @min 0.2
 * @max 1.0
 * @decimal 2
 * @increment 0.05
 * @group Wacm Attitude Control
 */
PARAM_DEFINE_FLOAT(WACM_R_TC, 0.4f);

/**
 * Attitude pitch time constant
 *
 * This defines the latency between a pitch step input and the achieved setpoint
 * (inverse to a P gain). Smaller systems may require smaller values.
 *
 * @unit s
 * @min 0.2
 * @max 1.0
 * @decimal 2
 * @increment 0.05
 * @group Wacm Attitude Control
 */
PARAM_DEFINE_FLOAT(WACM_P_TC, 0.4f);

/**
 * Maximum positive / up pitch rate setpoint
 *
 * @unit deg/s
 * @min 0.0
 * @max 180
 * @decimal 1
 * @increment 0.5
 * @group Wacm Attitude Control
 */
PARAM_DEFINE_FLOAT(WACM_P_RMAX_POS, 60.0f);

/**
 * Maximum negative / down pitch rate setpoint
 *
 * @unit deg/s
 * @min 0.0
 * @max 180
 * @decimal 1
 * @increment 0.5
 * @group Wacm Attitude Control
 */
PARAM_DEFINE_FLOAT(WACM_P_RMAX_NEG, 60.0f);

/**
 * Maximum roll rate setpoint
 *
 * @unit deg/s
 * @min 0.0
 * @max 180
 * @decimal 1
 * @increment 0.5
 * @group Wacm Attitude Control
 */
PARAM_DEFINE_FLOAT(WACM_R_RMAX, 70.0f);

/**
 * Maximum yaw rate setpoint
 *
 * @unit deg/s
 * @min 0.0
 * @max 180
 * @decimal 1
 * @increment 0.5
 * @group Wacm Attitude Control
 */
PARAM_DEFINE_FLOAT(WACM_Y_RMAX, 50.0f);
