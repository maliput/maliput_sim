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

from maliput_sim.core.utilities import *
from maliput_sim.core.components import *


class Entity:
    """
    Represents an entity in an Entity Component architecture.
    An entity has an ID and a collection of components.
    """

    def __init__(self, entity_id):
        """Initialize the entity."""
        self._entity_id = entity_id
        self._components = {}

    def add_component(self, component):
        """Add a component to the entity."""
        component_type = type(component)
        if component_type not in self._components:
            self._components[component_type] = []
        self._components[component_type].append(component)
        component.set_entity(self)

    def get_components(self, component_type):
        """Get all components of the specified type."""
        return self._components.get(component_type, [])

    def update(self, delta_time, sim_state, ecm):
        """Update the entity."""
        for component_list in self._components.values():
            for component in component_list:
                component.update(delta_time, sim_state, self, ecm)


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

    def update(self, delta_time, sim_state):
        """Forward the update() call to all entities."""
        for entity in self.entities.values():
            entity.update(delta_time, sim_state, self)
