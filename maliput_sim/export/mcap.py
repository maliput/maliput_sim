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

from maliput.api import RoadNetwork as MaliputRoadNetwork


class MCAP(Exporter):
    """
    This class dumps the simulation states into a mcap file that can be
    visualized using Foxglove Studio.

    Each simulation state is dumped to messages in the MCAP format.

    Serialization:
      MCAP supports several serialization formats. This class relies on the protobuf format.

    Channels:
      MCAP works on a publish-subscribe model. Each message is published to a channel.
      - /scene_update:
          This channel uses the SceneUpdate message type. See https://foxglove.dev/docs/studio/messages/scene-update.
          Foxglove Studio has a built-in viewer for this message type. So it is recommended to use this
          channel for the 3D visualization. The scene_update message contains a list of entities. Each entity can have
          multiple models (primitive shapes or even 3D models).
          This implementation uses:
            - One entity for the road network which loads the road network as a 3D model.
            - One entity for each agent which loads a cube as a 3D model.
      - /<agent_name>/pose:
          This channel uses the Pose message type to indicate the pose of the agent.
          See https://foxglove.dev/docs/studio/messages/pose.
      - /<agent_name>/behavior/<behavior_index>/<key>:
          This channel uses the KeyValuePair message type to indicate the values obtained from the behaviors' states
          in the agents. An agent can have multiple behaviors. Each behavior can have multiple parameters per state.
          The list of signals to export can be specified using the export_signals argument in the dump method and
          they should match the keys in the behavior state.
    """

    _DEFAULT_SIMULATION_OUTPUT_FILE_PATH = "simulation.mcap"

    _AGENT_BOX_SIZE = Vector3(x=4, y=2, z=1.5)
    _AGENT_COLOR = Color(r=1, g=0, b=0, a=0.8)

    _ROAD_NETWORK_ID = "road_network"
    _ROAD_NETWORK_COLOR = Color(r=0.2, g=0.8, b=0.9, a=0.1)
    _ROAD_NETWORK_MODEL_OFFSET = Vector3(x=0, y=0, z=0)
    _ROAD_NETWORK_MODEL_ROTATION = Quaternion(x=-0.70710678118, y=0, z=0, w=0.70710678118)
    _ROAD_NETWORK_MODEL_POSE = Pose(position=_ROAD_NETWORK_MODEL_OFFSET,
                                    orientation=_ROAD_NETWORK_MODEL_ROTATION)
    _ROAD_NETWORK_SCALE = Vector3(x=1, y=1, z=1)
    _ROAD_NETWORK_ENTITY_LIFE_TIME = Duration(seconds=0, nanos=0)

    _ROOT_FRAME_ID = "map"
    _MEDIA_TYPE = "model/obj"
    _SCENE_UPDATE_TOPIC = "/scene_update"

    def __init__(self, simulation: Simulation, output_file_path: str = ""):
        """
        Initialize the exporter.

        Args:
            simulation: The simulation instance to export.
            output_file_path: The path to the output file. If empty, the default path will be used.
        """
        self._output_file_path = (
            output_file_path if output_file_path else self._DEFAULT_SIMULATION_OUTPUT_FILE_PATH)
        super().__init__(simulation)

    def _add_road_network_entity_to_scene(self,
                                          road_network: MaliputRoadNetwork,
                                          scene_update: SceneUpdate,
                                          sim_time: int):
        """
        Add the road network entity to the scene_update as a model.

        Args:
            road_network: The maliput road network to export the model from.
            scene_update: The SceneUpdate message to add the entity to.
            sim_time: The simulation time in nanoseconds.
        """

        # Obtain the obj description from the road network.
        obj_description = get_obj_description_from_road_network(
            road_network).encode("utf-8")

        entity = scene_update.entities.add(
            lifetime=self._ROAD_NETWORK_ENTITY_LIFE_TIME
        )
        # Timestamp should match sim time.
        entity.timestamp.FromNanoseconds(sim_time)
        entity.frame_id = self._ROOT_FRAME_ID
        entity.id = self._ROAD_NETWORK_ID

        entity.models.add(
            color=self._ROAD_NETWORK_COLOR,
            override_color=True,
            pose=self._ROAD_NETWORK_MODEL_POSE,
            scale=self._ROAD_NETWORK_SCALE,
            media_type=self._MEDIA_TYPE,
            data=obj_description
        )

    def _add_agent_entity_to_scene(self, agent_name: str, pose: List[float], scene_update: SceneUpdate, sim_time: int):
        """
        Add the agent entity to the scene_update using a cube as visual.

        Args:
            agent_name: The name of the agent.
            pose: The pose of the agent.
            scene_update: The SceneUpdate message to add the entity to.
            sim_time: The simulation time in nanoseconds.
        """
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
             end_timestamp: float = 0.,
             export_signals: List = []):
        """
        Dump the simulation states into an mcap file.
        Ideally this method should be called after the simulation is complete.

        Args:
            export_agent_param: If True, the agent information will be exported.
            export_road_network_param: If True, the road network information will be exported.
            init_timestamp: The initial timestamp of the simulation in seconds.
            end_timestamp: The end timestamp of the simulation in seconds.
                           If 0, the simulation will be dumped until the end.
            export_signals: List of controller signals to export.
        """
        init_timestamp_ns = int(init_timestamp * 1e9)
        end_timestamp_ns = int(end_timestamp * 1e9)
        if end_timestamp_ns != 0 and end_timestamp_ns < init_timestamp_ns:
            raise ValueError("end_timestamp must be equal or greater than init_timestamp")
        with open(self._output_file_path, "wb") as f, Writer(f) as writer:
            scene_update = SceneUpdate()
            sim_states = self._simulation.get_sim_states()

            sim_time_ns = int(sim_states[0].sim_time * 1e9)
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

                sim_time_ns = int(sim_state.sim_time * 1e9)
                # Skip if sim_time is smaller than init_timestamp
                if (sim_time_ns < init_timestamp_ns):
                    continue
                # Stop if sim_time is greater than end_timestamp
                if (end_timestamp_ns != 0 and sim_time_ns > end_timestamp_ns):
                    break

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
                            topic="/" + name + "/pose",
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
                                        topic="/" + name + "/behavior/" + str(i) + "/" + key,
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
