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

from typing import List, Dict, Any

from maliput_sim.core.components import Component, Type
from maliput_sim.core.utilities import IDProvider

import maliput_sim


class Entity:
    """
    Represents an entity in an Entity Component architecture.
    An entity has an ID and a collection of components.
    """

    def __init__(self, entity_id: str):
        """Initialize the entity."""
        self._entity_id = entity_id
        self._components: Dict[type, List[Component]] = {}

    def add_component(self, component: Component):
        """Add a component to the entity."""
        component_type = type(component)
        if component_type not in self._components:
            self._components[component_type] = []
        self._components[component_type].append(component)
        component.set_entity(self)

    def get_components(self, component_type: type) -> List[Component]:
        """Get all components of the specified type."""
        return self._components.get(component_type, [])

    def update(self, delta_time: float,
               sim_state: 'maliput_sim.core.sim.SimulationState',
               ecm: 'EntityComponentManager') -> None:
        """Update the entity."""
        for component_list in self._components.values():
            for component in component_list:
                component.update(delta_time, sim_state, self, ecm)

    def get_state(self) -> Dict[str, Any]:
        """Get the state of the entity."""
        state: Dict[str, Any] = {
            'id': self._entity_id,
            'components': {}
        }
        for component_type, component_list in self._components.items():
            state['components'][component_type.__name__] = [component.get_state() for component in component_list]

        return state


class EntityComponentManager:
    """Manages the entities in the simulation."""

    def __init__(self):
        """Initialize the entity component manager."""
        self.entities = {}

    def create_entity(self):
        """Create a new entity."""
        entity_id = IDProvider.new_id()
        entity = Entity(entity_id)
        self.entities[entity_id] = entity
        return entity

    def get_entities(self):
        """Get all entities."""
        return self.entities

    def get_entity(self, entity_id):
        """Get the entity with the specified ID."""
        return self.entities[entity_id]

    def get_entities_with_component(self, component_type):
        """
        Get all entities that have a component of the specified type.
        This method doesn't take into account the components under a component container.
        """
        return list(filter(lambda entity: entity.get_components(component_type), self.entities.values()))

    def get_entities_of_type(self, entity_type):
        """Get all entities of the specified type."""
        entities = self.get_entities_with_component(Type)
        return list(filter(lambda entity: entity.get_components(Type)[0].get_type() == entity_type, entities))

    def update(self, delta_time, sim_state):
        """Forward the update() call to all entities."""
        for entity in self.entities.values():
            entity.update(delta_time, sim_state, self)

    def get_state(self):
        """Get the state of the entity component manager."""
        return {
            'entities': [entity.get_state() for entity in self.entities.values()]
        }
