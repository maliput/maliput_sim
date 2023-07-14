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

from maliput_sim.core.components import Component, ComponentContainer, Name, Type, Pose, Velocity, RoadNetwork
from maliput_sim.core.ecm import Entity, EntityComponentManager
from maliput_sim.core.sim import SimulationState

from maliput.api import RoadNetwork as MaliputRoadNetwork


class MockComponent(Component):
    def __init__(self, state=None):
        super().__init__()
        self._state = state
        self._nr_updates = 0

    def update(self, delta_time, sim_state, entity, ecm):
        self._nr_updates += 1

    def get_state(self):
        return self._state


class TestComponentBase:
    _ENTITY_ID = 1
    _SIM_TIME = 0.1


class TestComponent(unittest.TestCase, TestComponentBase):

    def test_set_entity(self):
        component = Component()
        entity = Entity(self._ENTITY_ID)
        component.set_entity(entity)
        self.assertEqual(component._entity, entity)

    def test_update(self):
        component = MockComponent()
        delta_time = 0.1
        sim_state = SimulationState(self._SIM_TIME, {})
        entity = Entity(self._ENTITY_ID)
        ecm = EntityComponentManager()
        self.assertEqual(component._nr_updates, 0)
        component.update(delta_time, sim_state, entity, ecm)
        self.assertEqual(component._nr_updates, 1)

    def test_get_state(self):
        component = MockComponent()
        self.assertIsNone(component.get_state())
        expected_state = "Test State"
        component = MockComponent(expected_state)
        self.assertEqual(component.get_state(), expected_state)


class TestComponentContainer(unittest.TestCase, TestComponentBase):

    class AnotherMockComponent(MockComponent):
        def __init__(self, state=None):
            super().__init__(state)

    def test_add_get_component(self):
        container = ComponentContainer()
        component_1 = MockComponent()
        component_2 = MockComponent()
        container.add_component(component_1)
        components = container.get_component(type(component_1))
        self.assertEqual(len(components), 1)
        container.add_component(component_2)
        self.assertEqual(len(components), 2)
        self.assertIn(component_1, components)
        self.assertIn(component_2, components)
        another_component_1 = self.AnotherMockComponent()
        another_component_2 = self.AnotherMockComponent()
        container.add_component(another_component_1)
        container.add_component(another_component_2)
        components = container.get_component(type(another_component_1))
        self.assertEqual(len(components), 2)
        self.assertIn(another_component_1, components)
        self.assertIn(another_component_2, components)

    def test_update(self):
        container = ComponentContainer()
        component = MockComponent()
        another_component = self.AnotherMockComponent()
        container.add_component(component)
        container.add_component(another_component)
        delta_time = 0.1
        sim_state = SimulationState(self._SIM_TIME, {})
        entity = Entity(self._ENTITY_ID)
        ecm = EntityComponentManager()
        container.update(delta_time, sim_state, entity, ecm)
        self.assertEqual(component._nr_updates, 1)
        self.assertEqual(another_component._nr_updates, 1)

    def test_get_state(self):
        container = ComponentContainer()
        component_state_1 = "Mock Component State 1"
        component_state_2 = "Mock Component State 2"
        another_component_state_1 = "Another Mock Component State 1"
        another_component_state_2 = "Another Mock Component State 2"
        component_1 = MockComponent(component_state_1)
        component_2 = MockComponent(component_state_2)
        another_component_1 = self.AnotherMockComponent(another_component_state_1)
        another_component_2 = self.AnotherMockComponent(another_component_state_2)
        container.add_component(component_1)
        container.add_component(component_2)
        container.add_component(another_component_1)
        container.add_component(another_component_2)

        state = container.get_state()
        print(state)
        expected_state = {type(component_1).__name__: [component_state_1, component_state_2],
                          type(another_component_1).__name__: [another_component_state_1, another_component_state_2]}
        self.assertEqual(state, expected_state)


class TestName(unittest.TestCase):

    def test_get_state(self):
        name = "Test Name"
        name_component = Name(name)
        state = name_component.get_state()
        self.assertEqual(state, name)

    def test_get_name(self):
        name = "Test Name"
        name_component = Name(name)
        self.assertEqual(name_component.get_name(), name)


class TestType(unittest.TestCase):

    def test_get_state(self):
        entity_type = "Test Type"
        type_component = Type(entity_type)
        state = type_component.get_state()
        self.assertEqual(state, entity_type)

    def test_get_type(self):
        entity_type = "Test Type"
        type_component = Type(entity_type)
        self.assertEqual(type_component.get_type(), entity_type)


class TestPose(unittest.TestCase):
    def setUp(self):
        self._position = (1, 2, 3)
        self._rotation = (4, 5, 6)
        self._dut = Pose(self._position, self._rotation)
        return super().setUp()

    def test_get_position(self):
        self.assertEqual(self._dut.get_position(), self._position)

    def test_get_state(self):
        state = self._dut.get_state()
        expected_state = self._position + self._rotation
        self.assertEqual(state, expected_state)


class TestVelocity(unittest.TestCase):
    def setUp(self):
        self._linear_velocity = (1, 2, 3)
        self._angular_velocity = (4, 5, 6)
        self._dut = Velocity(self._linear_velocity, self._angular_velocity)
        return super().setUp()

    def test_get_linear_velocity(self):
        self.assertEqual(self._dut.get_linear_vel(), self._linear_velocity)

    def test_get_angular_velocity(self):
        self.assertEqual(self._dut.get_angular_vel(), self._angular_velocity)

    def test_get_state(self):
        state = self._dut.get_state()
        expected_state = self._linear_velocity + self._angular_velocity
        self.assertEqual(state, expected_state)


class TestRoadNetwork(unittest.TestCase):

    class MockRoadNetwork(MaliputRoadNetwork):
        def __init__(self):
            pass

    def test_get_state(self):
        road_network = RoadNetwork(self.MockRoadNetwork())
        state = road_network.get_state()
        self.assertIsNone(state)

    def test_get_road_network(self):
        mock_rn = self.MockRoadNetwork()
        road_network = RoadNetwork(mock_rn)
        self.assertEqual(road_network.get_road_network(), mock_rn)
