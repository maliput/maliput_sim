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


class TestComponent(unittest.TestCase):

    def test_set_entity(self):
        component = Component()
        entity = Entity()
        component.set_entity(entity)
        self.assertEqual(component.entity, entity)

    def test_update(self):
        component = Component()
        delta_time = 0.1
        sim_state = SimState()
        entity = Entity()
        ecm = ECM()
        component.update(delta_time, sim_state, entity, ecm)
        # Add assertions for expected behavior

    def test_get_state(self):
        component = Component()
        state = component.get_state()
        self.assertIsNone(state)


class TestComponentContainer(unittest.TestCase):

    def test_add_component(self):
        container = ComponentContainer()
        component = Component()
        container.add_component(component)
        components = container.get_component(type(component))
        self.assertIn(component, components)

    def test_get_component(self):
        container = ComponentContainer()
        component = Component()
        container.add_component(component)
        components = container.get_component(type(component))
        self.assertIn(component, components)

    def test_update(self):
        container = ComponentContainer()
        component = Component()
        container.add_component(component)
        delta_time = 0.1
        sim_state = SimState()
        entity = Entity()
        ecm = ECM()
        container.update(delta_time, sim_state, entity, ecm)
        # Add assertions for expected behavior

    def test_get_state(self):
        container = ComponentContainer()
        component = Component()
        container.add_component(component)
        state = container.get_state()
        # Add assertions for expected state


class TestName(unittest.TestCase):

    def test_get_state(self):
        name = "Test Name"
        name_component = Name(name)
        state = name_component.get_state()
        self.assertEqual(state, name)


class TestType(unittest.TestCase):

    def test_get_state(self):
        entity_type = "Test Type"
        type_component = Type(entity_type)
        state = type_component.get_state()
        self.assertEqual(state, entity_type)


class TestPose(unittest.TestCase):

    def test_get_state(self):
        position = (1, 2, 3)
        rotation = (0, 0, 0)
        pose_component = Pose(position, rotation)
        state = pose_component.get_state()
        expected_state = position + rotation
        self.assertEqual(state, expected_state)


class TestVelocity(unittest.TestCase):

    def test_get_state(self):
        linear_velocity = (1, 0, 0)
        angular_velocity = (0, 0, 1)
        velocity_component = Velocity(linear_velocity, angular_velocity)
        state = velocity_component.get_state()
        expected_state = linear_velocity + angular_velocity
        self.assertEqual(state, expected_state)


class TestRoadNetwork(unittest.TestCase):

    def test_get_state(self):
        road_network = RoadNetwork()
        state = road_network.get_state()
        self.assertIsNone(state)


if __name__ == '__main__':
    unittest.main()
