/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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
 * @file groundroll.cpp
 *
 * Helper class for automatic ground roll / water taxi navigation
 *
 * @author PX4 Development Team
 */

#include "groundroll.h"
#include "navigator.h"

#include <lib/mathlib/mathlib.h>
#include <uORB/topics/vehicle_global_position.h>

GroundRoll::GroundRoll(Navigator *navigator) :
	MissionBase(navigator, 10)
{
}

void
GroundRoll::on_activation()
{
	_waypoint_position_reached = false;
	_waypoint_yaw_reached = false;
	_navigator->get_position_setpoint_triplet()->current.valid = false;
	_navigator->set_position_setpoint_triplet_updated();
	_mission_has_been_activated = true;
}

void
GroundRoll::on_active()
{
	if (_navigator->get_vstatus()->arming_state != vehicle_status_s::ARMING_STATE_ARMED) {
		return;
	}

	if (_waypoint_position_reached) {
		advance_to_next_waypoint();
		return;
	}

	if (is_waypoint_reached()) {
		_waypoint_position_reached = true;
		_time_wp_reached = hrt_absolute_time();
		return;
	}

	set_groundroll_position_setpoint();
}

void
GroundRoll::setActiveMissionItems()
{
	if (loadCurrentMissionItem()) {
		mission_item_to_position_setpoint(_mission_item, &_navigator->get_position_setpoint_triplet()->current);
		_navigator->get_position_setpoint_triplet()->current.type = position_setpoint_s::SETPOINT_TYPE_GROUNDROLL;
		_navigator->get_position_setpoint_triplet()->current.acceptance_radius = GROUNDROLL_ACCEPTANCE_RADIUS;
		_navigator->set_position_setpoint_triplet_updated();
	}
}

bool
GroundRoll::setNextMissionItem()
{
	return goToNextItem(true) == PX4_OK;
}

void
GroundRoll::set_groundroll_position_setpoint()
{
	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	pos_sp_triplet->previous.lat = _navigator->get_global_position()->lat;
	pos_sp_triplet->previous.lon = _navigator->get_global_position()->lon;
	pos_sp_triplet->previous.alt = _navigator->get_global_position()->alt;
	pos_sp_triplet->previous.yaw = _navigator->get_local_position()->heading;
	pos_sp_triplet->previous.valid = true;

	mission_item_to_position_setpoint(_mission_item, &pos_sp_triplet->current);

	pos_sp_triplet->current.type = position_setpoint_s::SETPOINT_TYPE_GROUNDROLL;
	pos_sp_triplet->current.acceptance_radius = GROUNDROLL_ACCEPTANCE_RADIUS;

	pos_sp_triplet->next.valid = false;

	_navigator->set_position_setpoint_triplet_updated();
}

bool
GroundRoll::is_waypoint_reached()
{
	if (!item_contains_position(_mission_item)) {
		return false;
	}

	const float dist = get_distance_to_next_waypoint(
				   _mission_item.lat, _mission_item.lon,
				   _navigator->get_global_position()->lat, _navigator->get_global_position()->lon);

	return dist < GROUNDROLL_ACCEPTANCE_RADIUS;
}

void
GroundRoll::advance_to_next_waypoint()
{
	if (goToNextItem(true) == PX4_OK) {
		_waypoint_position_reached = false;
		_waypoint_yaw_reached = false;
		set_groundroll_position_setpoint();

	} else {
		position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
		pos_sp_triplet->previous.valid = false;
		pos_sp_triplet->current.valid = false;
		pos_sp_triplet->next.valid = false;
		_navigator->set_position_setpoint_triplet_updated();

		_navigator->get_mission_result()->finished = true;
		_navigator->set_mission_result_updated();
	}
}