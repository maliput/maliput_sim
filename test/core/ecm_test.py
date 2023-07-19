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

from maliput_sim.core.ecm import Entity, EntityComponentManager, Type

from unittest.mock import MagicMock


class TestEntity(unittest.TestCase):

    def setUp(self):
        self.entity_id = "entity1"
        self.entity = Entity(self.entity_id)

    def test_add_component(self):
        component = MagicMock()
        self.entity.add_component(component)

        components = self.entity.get_components(type(component))
        self.assertIn(component, components)

    def test_get_components(self):
        component1 = MagicMock()
        self.entity.add_component(component1)

        components = self.entity.get_components(type(component1))
        self.assertIn(component1, components)

    def test_update(self):
        component1 = MagicMock()
        component2 = MagicMock()
        self.entity.add_component(component1)
        self.entity.add_component(component2)

        delta_time = 0.1
        sim_state = MagicMock()
        ecm = MagicMock()
        self.entity.update(delta_time, sim_state, ecm)

        component1.update.assert_called_once_with(delta_time, sim_state, self.entity, ecm)
        component2.update.assert_called_once_with(delta_time, sim_state, self.entity, ecm)

    def test_get_state(self):
        class ComponentA(MagicMock):
            pass

        class ComponentB(MagicMock):
            pass

        component_a = ComponentA()
        component_a.get_state.return_value = {"key": "value1"}
        component_b = ComponentB()
        component_b.get_state.return_value = {"key": "value2"}
        self.entity.add_component(component_a)
        self.entity.add_component(component_b)

        expected_state = {
            'id': self.entity_id,
            'components': {
                'ComponentA': [{"key": "value1"}],
                'ComponentB': [{"key": "value2"}]
            }
        }
        self.assertEqual(self.entity.get_state(), expected_state)


class TestEntityComponentManager(unittest.TestCase):

    def setUp(self):
        self.dut = EntityComponentManager()

    def test_create_entity(self):
        entity = self.dut.create_entity()
        self.assertIsInstance(entity, Entity)
        self.assertIsNotNone(entity._entity_id)  # Ensure entity ID is assigned

    def test_get_entities(self):
        entities = self.dut.get_entities()
        self.assertEqual(len(entities), 0)

        entity1 = self.dut.create_entity()
        entity2 = self.dut.create_entity()

        entities = self.dut.get_entities()
        self.assertEqual(len(entities), 2)
        self.assertIn(entity1._entity_id, entities)
        self.assertIn(entity2._entity_id, entities)

    def test_get_entity(self):
        entity1 = self.dut.create_entity()
        entity2 = self.dut.create_entity()

        retrieved_entity1 = self.dut.get_entity(entity1._entity_id)
        retrieved_entity2 = self.dut.get_entity(entity2._entity_id)

        self.assertEqual(entity1, retrieved_entity1)
        self.assertEqual(entity2, retrieved_entity2)

    def test_get_entities_with_component(self):
        component_type = type(MagicMock())
        entity1 = self.dut.create_entity()
        entity2 = self.dut.create_entity()
        entity3 = self.dut.create_entity()

        entity1.get_components = MagicMock(return_value=[MagicMock()])
        entity2.get_components = MagicMock(return_value=[])
        entity3.get_components = MagicMock(return_value=[MagicMock()])
        print(entity1.get_components())
        print(entity2.get_components())
        print(entity3.get_components())
        entities_with_component = self.dut.get_entities_with_component(component_type)
        self.assertEqual(len(entities_with_component), 2)
        self.assertIn(entity1, entities_with_component)
        self.assertIn(entity3, entities_with_component)

    def test_get_entities_of_type(self):
        entity1 = self.dut.create_entity()
        entity_type = "TestType"
        entity1.add_component(Type(entity_type))

        entity2 = self.dut.create_entity()
        different_entity_type = "DifferentType"
        entity2.add_component(Type(different_entity_type))

        entity3 = self.dut.create_entity()

        entities_of_type = self.dut.get_entities_of_type(entity_type)
        self.assertEqual(len(entities_of_type), 1)
        self.assertIn(entity1, entities_of_type)
        self.assertNotIn(entity2, entities_of_type)
        self.assertNotIn(entity3, entities_of_type)

    def test_update(self):
        entity1 = self.dut.create_entity()
        entity2 = self.dut.create_entity()

        entity1.update = MagicMock()
        entity2.update = MagicMock()

        delta_time = 0.1
        sim_state = MagicMock()

        self.dut.update(delta_time, sim_state)

        entity1.update.assert_called_once_with(delta_time, sim_state, self.dut)
        entity2.update.assert_called_once_with(delta_time, sim_state, self.dut)

    def test_get_state(self):
        entity1 = self.dut.create_entity()
        entity1.get_state = MagicMock(return_value={"state": "entity1"})
        entity2 = self.dut.create_entity()
        entity2.get_state = MagicMock(return_value={"state": "entity2"})

        state = self.dut.get_state()
        self.assertEqual(len(state['entities']), 2)
        self.assertIn({"state": "entity1"}, state['entities'])
        self.assertIn({"state": "entity2"}, state['entities'])
