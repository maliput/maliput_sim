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

import unittest
from unittest.mock import MagicMock

from maliput_sim.core.components import Name, Type, Pose, Velocity
from maliput_sim.simulation import Simulation, SimulationConfig, AgentInitialState, EntityComponentManager


class TestSimulation(unittest.TestCase):
    _AGENT_TYPE = "agent"
    _ROAD_NETWORK_TYPE = "road_network"

    def setUp(self):
        self.road_network = MagicMock()
        self.sim_config = SimulationConfig(real_time_factor=1.0, time_step=0.1)
        self.simulation = Simulation(self.road_network, self.sim_config)

    def test_init(self):
        # Check if the simulation state is initialized correctly
        sim_state = self.simulation.get_sim_state()
        self.assertEqual(sim_state.sim_time, 0.0)
        self.assertEqual(sim_state.ecm_state, {})

        # Check if the simulation config is initialized correctly
        sim_config = self.simulation.get_sim_config()
        self.assertEqual(sim_config.real_time_factor, 1.0)
        self.assertEqual(sim_config.time_step, 0.1)

        # Check if the road network is initialized correctly
        road_network = self.simulation.get_road_network()
        self.assertEqual(road_network, self.road_network)

        # Check ecm is initialized correctly
        ecm = self.simulation.get_ecm()
        self.assertIsInstance(ecm, EntityComponentManager)
        self.assertEqual(len(ecm.get_entities()), 1)
        self.assertEqual(len(ecm.get_entities_of_type(self._ROAD_NETWORK_TYPE)), 1)

    def test_add_agent(self):
        self._add_agent("Agent1")

        ecm = self.simulation.get_ecm()
        self.assertEqual(len(ecm.get_entities()), 2)

        agent_entity = ecm.get_entities_of_type("agent")[0]
        self.assertEqual(agent_entity.get_components(Name)[0].get_state(), "Agent1")
        self.assertEqual(agent_entity.get_components(Type)[0].get_state(), "agent")

    def test_step(self):
        self._add_agent("Agent1")

        # Simulating one step
        self.simulation.step()

        # Check if the simulation state is updated
        sim_state = self.simulation.get_sim_state()
        self.assertAlmostEqual(sim_state.sim_time, 0.1)
        self.assertEqual(len(sim_state.ecm_state), 1)
        self.assertIn("entities", sim_state.ecm_state)

        # Check if the agent's state is updated
        ecm = self.simulation.get_ecm()
        entities = ecm.get_entities_of_type(self._AGENT_TYPE)
        self.assertEqual(len(entities), 1)
        agent_entity = entities[0]
        agent_pose = agent_entity.get_components(Pose)[0].get_state()
        self.assertAlmostEqual(agent_pose[0], 0.2)  # Linear x position after 1 step

    def test_step_for(self):
        self._add_agent("Agent1")

        # Simulating for 2 seconds
        self.simulation.step_for(duration=2.0)

        # Check if the simulation state is updated
        sim_state = self.simulation.get_sim_state()
        self.assertAlmostEqual(sim_state.sim_time, 2.0)

        # Check if the agent's state is updated
        ecm = self.simulation.get_ecm()
        entities = ecm.get_entities_of_type(self._AGENT_TYPE)
        self.assertEqual(len(entities), 1)
        agent_entity = entities[0]
        agent_pose = agent_entity.get_components(Pose)[0].get_state()
        self.assertAlmostEqual(agent_pose[0], 4.0)  # Linear x position after 2 seconds

    def test_get_sim_states(self):
        self._add_agent("Agent1")

        # Simulating for 2 seconds
        self.simulation.step_for(duration=2.0)

        # Check if all the states of the simulation are returned
        sim_states = self.simulation.get_sim_states()
        self.assertEqual(len(sim_states), 21)  # 20 steps + initial state

    def _move_forward_controller(self, duration, sim_state, entity, ecm):
        """Agent controller."""
        pose = entity.get_components(Pose)[0]
        velocity = entity.get_components(Velocity)[0]

        # Move the agent forward in X direction using the current velocity.
        pose._position[0] += duration * velocity.get_linear_vel()[0]

    def _dumb_get_state(self):
        return {"key": "value"}

    def _add_agent(self, name):
        initial_state = AgentInitialState(name=name, position=[0, 0, 0],
                                          rotation=[0, 0, 0, 0], linear_vel=2., angular_vel=0.)
        controller = self._move_forward_controller
        get_state = self._dumb_get_state
        self.simulation.add_agent(initial_state, controller, get_state)
