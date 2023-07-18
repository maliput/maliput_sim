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
from maliput_sim.core.sim import (Behavior, SimulationConfig,
                         SimulationState, AgentInitialState)


class TestSimulationConfig(unittest.TestCase):

    def test_default_config(self):
        config = SimulationConfig()
        self.assertEqual(config.real_time_factor, 1.0)
        self.assertEqual(config.time_step, 0.01)

    def test_custom_config(self):
        config = SimulationConfig(real_time_factor=2.0, time_step=0.05)
        self.assertEqual(config.real_time_factor, 2.0)
        self.assertEqual(config.time_step, 0.05)


class TestAgentInitialState(unittest.TestCase):

    def test_agent_initial_state(self):
        initial_state = AgentInitialState(name="Agent1", position=[1, 2, 3],
                                          rotation=[0, 0, 0, 0], linear_vel=2.0, angular_vel=1.0)
        self.assertEqual(initial_state.name, "Agent1")
        self.assertEqual(initial_state.position, [1, 2, 3])
        self.assertEqual(initial_state.rotation, [0, 0, 0, 0])
        self.assertEqual(initial_state.linear_vel, 2.0)
        self.assertEqual(initial_state.angular_vel, 1.0)


class TestSimulationState(unittest.TestCase):

    def test_simulation_state(self):
        ecm_state = {"entity1": {"component1": {"key": "value"}}}
        sim_state = SimulationState(sim_time=10.0, ecm_state=ecm_state)

        self.assertEqual(sim_state.sim_time, 10.0)
        self.assertEqual(sim_state.ecm_state, ecm_state)

class TestBehavior(unittest.TestCase):

    def test_behavior_update(self):
        # Mocking the behavior function
        behavior_mock = MagicMock()

        # Creating a Behavior instance with the mocked behavior
        behavior_component = Behavior(behavior=behavior_mock)

        # Mocking the necessary parameters for update()
        delta_time = 0.1
        sim_state = MagicMock()
        entity = MagicMock()
        ecm = MagicMock()

        # Calling the update() method
        behavior_component.update(delta_time, sim_state, entity, ecm)

        # Assert that the behavior function is called
        behavior_mock.assert_called_once_with(delta_time, sim_state, entity, ecm)

    def test_behavior_get_state(self):
        # Mocking the get_state function
        get_state_mock = MagicMock(return_value={"key": "value"})

        # Creating a Behavior instance with the mocked get_state function
        behavior_component = Behavior(behavior=MagicMock(), get_state=get_state_mock)

        # Calling the get_state() method
        state = behavior_component.get_state()

        # Assert that the get_state function is called and returns the expected state
        get_state_mock.assert_called_once()
        self.assertEqual(state, {"key": "value"})
