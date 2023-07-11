# BSD 3-Clause License
#
# Copyright (c) 2023, Woven by Toyota
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

from enum import Enum

from maliput.api import Lane, LanePosition, InertialPosition, Which
from maliput_sim.core.components import Pose, Velocity, RoadNetwork


class RailCarController():
    class LaneDirection(Enum):
        FORWARD = "0"
        BACKWARD = "1"

    _LANE_DIRECTION_TO_INT_MAP = {LaneDirection.FORWARD: 1, LaneDirection.BACKWARD: -1}
    _LANE_END_TO_LANE_DIRECTION_MAP = {Which.kStart: LaneDirection.FORWARD, Which.kFinish: LaneDirection.BACKWARD}

    def __init__(self):
        self._lane_direction = RailCarController.LaneDirection.FORWARD
        self._lane_position: LanePosition
        self._lane: Lane
        self._inertial_position: InertialPosition

    def step(self, duration, sim_state, entity, ecm):
        """Agent controller."""
        pose = entity.get_components(Pose)[0]
        velocity = entity.get_components(Velocity)[0]

        # Move the agent forward in _s_ direction using the current velocity.

        rn_entity = ecm.get_entities_of_type("road_network")[0]
        rn = rn_entity.get_components(RoadNetwork)[0].road_network
        rg = rn.road_geometry()
        road_position_result = rg.ToRoadPosition(InertialPosition(
            pose.position[0], pose.position[1], pose.position[2]))
        road_position = road_position_result.road_position

        s_increment = duration * velocity.linear

        self._lane = road_position.lane
        new_s = road_position.pos.s() + self._LANE_DIRECTION_TO_INT_MAP.get(self._lane_direction) * s_increment
        if (new_s > road_position.lane.length() or new_s < 0):
            new_lane_end = RailCarController._get_next_lane_end(
                road_position.lane, self._lane_direction)
            if (new_lane_end is None):
                return
            self._lane_direction = self._LANE_END_TO_LANE_DIRECTION_MAP.get(new_lane_end.end)
            self._lane = new_lane_end.lane
            new_s = 0 if self._lane_direction == RailCarController.LaneDirection.FORWARD else self._lane.length()

        self._lane_position = LanePosition(new_s, 0., 0.)
        self._inertial_position = self._lane.ToInertialPosition(self._lane_position)
        new_orientation = self._lane.GetOrientation(self._lane_position).quat()
        pose.position[0] = self._inertial_position.x()
        pose.position[1] = self._inertial_position.y()
        pose.position[2] = self._inertial_position.z()
        pose.rotation[0] = new_orientation.x()
        pose.rotation[1] = new_orientation.y()
        pose.rotation[2] = new_orientation.z()
        pose.rotation[3] = new_orientation.w()

    def get_state(self):
        state = {}
        state["id"] = type(self).__name__
        state["lane_id"] = self._lane.id().string()
        state["lane_position"] = self._lane_position.srh().__str__()
        state["inertial_position"] = self._inertial_position.xyz().__str__()
        return state

    def _get_next_lane_end(lane, lane_direction):
        return lane.GetDefaultBranch(
            Which.kFinish if lane_direction == RailCarController.LaneDirection.FORWARD else Which.kStart)
