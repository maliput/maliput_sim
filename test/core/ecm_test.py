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

from maliput_sim.core.ecm import Entity

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
