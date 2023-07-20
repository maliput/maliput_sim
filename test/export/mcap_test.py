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
import os
import unittest

import shutil
from unittest.mock import MagicMock
from typing import List
from tempfile import mkdtemp

from maliput_sim.core.sim import SimulationState
from maliput_sim.simulation import Simulation
from maliput_sim.export.mcap import MCAP

from foxglove_schemas_protobuf.SceneUpdate_pb2 import SceneUpdate
from foxglove_schemas_protobuf.Pose_pb2 import Pose
from foxglove_schemas_protobuf.Quaternion_pb2 import Quaternion
from foxglove_schemas_protobuf.Vector3_pb2 import Vector3
from maliput.plugin import create_road_network
from mcap_protobuf.decoder import DecoderFactory
from mcap.reader import make_reader


def GetRoadNetwork():
    """
    Returns an arbitrary road network.
    """
    # TODO(#12): Rely on a road network loader once that is implemented.
    # Create road network model
    env = os.getenv("MALIPUT_MALIDRIVE_RESOURCE_ROOT").split(":")
    env = env[0]
    rn_configuration = {
        "opendrive_file": env + "/resources/odr/TShapeRoad.xodr" if env is not None else "",
        "road_geometry_id": "TShapeRoadRoadNetwork"
    }
    maliput_road_network = create_road_network(
        "maliput_malidrive", rn_configuration)
    return maliput_road_network


def GetSimulationMock(simulation_states: List[SimulationState], road_network):
    """
    Returns a mock simulation with the given simulation states and road network.

    Args:
        simulation_states: The simulation states to be returned by the mock simulation.
        road_network: The road network to be returned by the mock simulation.
    """
    mock_simulation = MagicMock(spec=Simulation)
    mock_simulation.get_sim_states.return_value = simulation_states
    mock_simulation.get_road_network.return_value = road_network
    return mock_simulation


def GetSimulationState(sim_time: float):
    """
    Returns a simulation state at a given simulation time.
    The ecm state contains a road network entity and an agent entity.
    The velocity of the agent is set to 1.0 in the x direction.
    """
    linear_vel_x = 1.
    return SimulationState(
        sim_time=sim_time,
        ecm_state={'entities':
                   [
                       {
                           'id': 'entity_1',
                           'components':
                           {
                               'Name': ['maliput'],
                               'Type': ['road_network'],
                               'RoadNetwork': [None]
                           }
                       },
                       {
                           'id': 'entity_2',
                           'components':
                           {
                               'Name': ['agent_1'],
                               'Type': ['agent'],
                               'Pose': [[linear_vel_x * sim_time, 0.0, 0.0, 0.0, 0.0, 0.0, 1]],
                               'Velocity': [[1.0, 0, 0, 0, 0, 0.0]],
                               'Behavior': [{'signal_1': 'value_1'}]
                           }
                       },
                   ]
                   })


class TestMCAP(unittest.TestCase):
    _SIM_STATE_EMPTY = SimulationState(sim_time=0.0, ecm_state={})

    def test_init_method(self):
        simulation = GetSimulationMock([self._SIM_STATE_EMPTY], None)
        dut = MCAP(simulation)
        self.assertEqual(dut._simulation, simulation)
        self.assertEqual(dut._output_file_path, MCAP._DEFAULT_SIMULATION_OUTPUT_FILE_PATH)
        dut = MCAP(simulation, "random_path.mcap")
        self.assertEqual(dut._output_file_path, "random_path.mcap")

    def test_add_road_network_entity_to_scene_method(self):
        road_network = GetRoadNetwork()
        simulation = GetSimulationMock([self._SIM_STATE_EMPTY], road_network)
        scene_update = SceneUpdate()
        sim_time = 12

        dut = MCAP(simulation)
        dut._add_road_network_entity_to_scene(road_network, scene_update, sim_time)
        self.assertEqual(len(scene_update.entities), 1)
        rn_entity = scene_update.entities[0]
        self.assertEqual(rn_entity.id, dut._ROAD_NETWORK_ID)
        self.assertEqual(rn_entity.timestamp.ToNanoseconds(), sim_time)
        self.assertEqual(rn_entity.lifetime, dut._ROAD_NETWORK_ENTITY_LIFE_TIME)
        self.assertEqual(rn_entity.frame_id, dut._ROOT_FRAME_ID)
        self.assertEqual(len(rn_entity.models), 1)
        rn_model = rn_entity.models[0]
        self.assertEqual(rn_model.media_type, dut._MEDIA_TYPE)
        self.assertEqual(rn_model.color, dut._ROAD_NETWORK_COLOR)
        self.assertEqual(rn_model.pose, dut._ROAD_NETWORK_MODEL_POSE)
        self.assertTrue(rn_model.data)

    def test_add_agent_entity_to_scene_method(self):
        road_network = GetRoadNetwork()
        simulation = GetSimulationMock([self._SIM_STATE_EMPTY], road_network)
        scene_update = SceneUpdate()
        sim_time = 12

        agent_name = "agent_1"
        pose = [1.0, 2.0, 3.0, 0.12, 0.24, 0.36, 0.8]
        dut = MCAP(simulation)
        dut._add_agent_entity_to_scene(agent_name, pose, scene_update, sim_time)
        self.assertEqual(len(scene_update.entities), 1)
        agent_entity = scene_update.entities[0]
        self.assertEqual(agent_entity.id, agent_name)
        self.assertEqual(agent_entity.timestamp.ToNanoseconds(), sim_time)
        self.assertEqual(agent_entity.frame_id, dut._ROOT_FRAME_ID)
        self.assertEqual(len(agent_entity.cubes), 1)
        agent_cube = agent_entity.cubes[0]
        self.assertEqual(agent_cube.size, dut._AGENT_BOX_SIZE)
        self.assertEqual(agent_cube.color, dut._AGENT_COLOR)
        pose = Pose(
            position=Vector3(x=pose[0], y=pose[1], z=pose[2]),
            orientation=Quaternion(
                x=pose[3], y=pose[4], z=pose[5], w=pose[6])
        )
        self.assertEqual(agent_cube.pose, pose)


class TestMCAPDumpMethod(unittest.TestCase):
    _FILE_NAME = "mcap_test.mcap"

    def setUp(self) -> None:
        sim = GetSimulationMock([GetSimulationState(0.), GetSimulationState(
            1.), GetSimulationState(2.)], GetRoadNetwork())
        self.tmp_dir = mkdtemp()
        self.obj_file_path = os.path.join(self.tmp_dir, self._FILE_NAME)
        self.dut = MCAP(sim, self.obj_file_path)
        return super().setUp()

    def tearDown(self) -> None:
        shutil.rmtree(self.tmp_dir)

    def test_dump_method_default_values(self):
        self.dut.dump()
        self.assertTrue(os.path.exists(self.obj_file_path))
        with open(self.obj_file_path, "rb") as f:
            reader = make_reader(f, decoder_factories=[DecoderFactory()])
            scene_update_msgs = list(
                reader.iter_decoded_messages(topics=["/scene_update"]))
            self.assertEqual(len(scene_update_msgs), 3)
            for schema, channel, message, proto_msg in scene_update_msgs:
                self.assertEqual(schema.name, "foxglove.SceneUpdate")
                # TODO: Check the content of the message.

            pose_msgs = list(reader.iter_decoded_messages(
                topics=["/agent_1/pose"]))
            # Pose msg isn't being published in first iteration.
            self.assertEqual(len(pose_msgs), 2)
            for schema, channel, message, proto_msg in pose_msgs:
                self.assertEqual(schema.name, "foxglove.Pose")
                # TODO: Check the content of the message.

    def test_dump_method_exporting_signals(self):
        self.dut.dump(export_signals=['signal_1'])
        self.assertTrue(os.path.exists(self.obj_file_path))
        with open(self.obj_file_path, "rb") as f:
            reader = make_reader(f, decoder_factories=[DecoderFactory()])
            behavior_msgs = list(reader.iter_decoded_messages(
                topics=["/agent_1/behavior/0/signal_1"]))
            # behavior msg isn't being published in first iteration.
            self.assertEqual(len(behavior_msgs), 2)
            for schema, channel, message, proto_msg in behavior_msgs:
                self.assertEqual(schema.name, "foxglove.KeyValuePair")
                self.assertEqual(proto_msg.key, "signal_1")
                self.assertEqual(proto_msg.value, "value_1")
