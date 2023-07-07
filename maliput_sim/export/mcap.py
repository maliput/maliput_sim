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

from typing import List

from maliput_sim.export.base import Exporter
from maliput_sim.simulation import Simulation
from maliput_sim.utils.obj import get_obj_description_from_road_network

from foxglove_schemas_protobuf.Color_pb2 import Color
from foxglove_schemas_protobuf.KeyValuePair_pb2 import KeyValuePair
from foxglove_schemas_protobuf.Pose_pb2 import Pose
from foxglove_schemas_protobuf.Quaternion_pb2 import Quaternion
from foxglove_schemas_protobuf.Vector3_pb2 import Vector3
from foxglove_schemas_protobuf.SceneUpdate_pb2 import SceneUpdate
from google.protobuf.duration_pb2 import Duration
from mcap_protobuf.writer import Writer


class MCAP(Exporter):
    """A Foxglove visualization. This class dumps the simulation states into a mcap file that can be
    visualized using Foxglove Studio."""

    _DEFAULT_SIMULATION_OUTPUT_FILE_PATH = "simulation.mcap"

    _AGENT_BOX_SIZE = Vector3(x=4, y=2, z=1.5)
    _AGENT_COLOR = Color(r=1, g=0, b=0, a=0.8)

    _ROAD_NETWORK_ID = "road_network"
    _ROAD_NETWORK_COLOR = Color(r=0.2, g=0.8, b=0.9, a=0.1)
    _ROAD_NETWORK_MODEL_OFFSET = Vector3(x=0, y=0, z=0)
    _ROAD_NETWORK_MODEL_ROTATION = Quaternion(x=-0.70710678118, y=0, z=0, w=0.70710678118)
    _ROAD_NETWORK_SCALE = Vector3(x=1, y=1, z=1)

    _ROOT_FRAME_ID = "map"
    _MEDIA_TYPE = "model/obj"
    _SCENE_UPDATE_TOPIC = "/scene_update"

    def __init__(self, simulation: Simulation, output_file_path: str = ""):
        self._output_file_path = (
            output_file_path if output_file_path else self._DEFAULT_SIMULATION_OUTPUT_FILE_PATH)
        super().__init__(simulation)

    def _add_road_network_entity_to_scene(self, road_network, scene_update, sim_time):
        """Add the road network entity to the scene_update as a model."""

        # Obtain the obj description from the road network.
        obj_description = get_obj_description_from_road_network(
            road_network).encode("utf-8")

        entity = scene_update.entities.add(
            lifetime=Duration(seconds=0, nanos=0)
        )
        # Timestamp should match sim time.
        entity.timestamp.FromNanoseconds(sim_time)
        entity.frame_id = self._ROOT_FRAME_ID
        entity.id = self._ROAD_NETWORK_ID

        entity.models.add(
            color=self._ROAD_NETWORK_COLOR,
            override_color=True,
            pose=Pose(
                position=self._ROAD_NETWORK_MODEL_OFFSET,
                orientation=self._ROAD_NETWORK_MODEL_ROTATION
            ),
            scale=self._ROAD_NETWORK_SCALE,
            media_type=self._MEDIA_TYPE,
            data=obj_description
        )

    def _add_agent_entity_to_scene(self, agent_name, pose, scene_update, sim_time):
        """Add the agent entity to the scene_update as a cube."""
        entity = scene_update.entities.add()
        entity.timestamp.FromNanoseconds(sim_time)
        entity.id = agent_name
        entity.frame_id = self._ROOT_FRAME_ID
        entity.cubes.add(
            size=self._AGENT_BOX_SIZE,
            pose=Pose(
                position=Vector3(x=pose[0], y=pose[1], z=pose[2]),
                orientation=Quaternion(
                    x=pose[3], y=pose[4], z=pose[5], w=pose[6])
            ),
            color=self._AGENT_COLOR,
        )

    def dump(self,
             export_agent_param: bool = True,
             export_road_network_param: bool = True,
             init_timestamp: float = 0.,
             export_signals: List = []):
        """
        Dump the simulation states into an mcap file.
        Ideally this method should be called after the simulation is complete.

        Args:
            export_agent_param: If True, the agent information will be exported.
            export_road_network_param: If True, the road network information will be exported.
            init_timestamp: The initial timestamp of the simulation in seconds.
            export_signals: TBD
        """

        with open(self._output_file_path, "wb") as f, Writer(f) as writer:
            scene_update = SceneUpdate()
            sim_states = self._simulation.get_sim_states()

            init_timestamp_ns = int(init_timestamp * 1e9) if init_timestamp is not None else 0
            sim_time_ns = int(sim_states[0].sim_time * 1e9) + init_timestamp_ns
            if (export_road_network_param):
                self._add_road_network_entity_to_scene(self._simulation.get_road_network(), scene_update, sim_time_ns)
            writer.write_message(
                topic=self._SCENE_UPDATE_TOPIC,
                message=scene_update,
                log_time=sim_time_ns,
                publish_time=sim_time_ns,
            )

            for sim_state in sim_states[1:]:
                if (sim_state.ecm_state is None):
                    continue

                sim_time_ns = int(sim_state.sim_time * 1e9) + init_timestamp_ns
                scene_update = SceneUpdate()
                entities = sim_state.ecm_state['entities']
                if (export_agent_param):
                    agent_entities = (
                        entity for entity in entities if 'agent' in entity.get('components', {}).get('Type', []))

                    # Add agent information
                    for sim_state_entity in agent_entities:
                        components = sim_state_entity['components']
                        name = components['Name'][0]
                        pose = components['Pose'][0]
                        # Add agent entity to scene_update
                        self._add_agent_entity_to_scene(name, pose, scene_update, sim_time_ns)

                        # Publish agent pose
                        pose_message = Pose(
                            position=Vector3(x=pose[0], y=pose[1], z=pose[2]),
                            orientation=Quaternion(x=pose[3], y=pose[4], z=pose[5], w=pose[6])
                        )
                        writer.write_message(
                            topic="/" + name + "/pose/",
                            message=pose_message,
                            log_time=sim_time_ns,
                            publish_time=sim_time_ns,
                        )
                        if (export_signals):
                            behaviors_state = components['Behavior']
                            filtered_behaviors = (
                                behavior_state for behavior_state in behaviors_state if any(
                                    key in export_signals for key in behavior_state))
                            for i, behavior_state in enumerate(filtered_behaviors):
                                for key, value in behavior_state.items():
                                    if key not in export_signals:
                                        continue
                                    key_value_pair_message = KeyValuePair(
                                        key=key,
                                        value=value
                                    )
                                    writer.write_message(
                                        topic="/" + name + "/behavior/" + str(i) + "/" + key + "/",
                                        message=key_value_pair_message,
                                        log_time=sim_time_ns,
                                        publish_time=sim_time_ns,
                                    )

                writer.write_message(
                    topic=self._SCENE_UPDATE_TOPIC,
                    message=scene_update,
                    log_time=sim_time_ns,
                    publish_time=sim_time_ns,
                )
