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

class Component:
    """Base class for components."""
    pass


class Name(Component):
    """A component that stores a name."""
    def __init__(self, name):
        self.name = name


class Type(Component):
    """
    A component that stores a type.
    Useful for distinguishing between different types of entities.
    """
    def __init__(self, type):
        self.type = type


class Pose(Component):
    """A component that stores a position and rotation."""
    def __init__(self, position, rotation):
        self.position = position
        self.rotation = rotation


class Velocity(Component):
    """A component that stores a linear and angular velocity."""
    def __init__(self, linear, angular):
        self.linear = linear
        self.angular = angular


class Controller(Component):
    """A component that stores a controller."""
    def __init__(self, controller):
        self.controller = controller


class RoadNetwork(Component):
    """A component that stores a road network."""
    def __init__(self, road_network):
        self.road_network = road_network


class Entity:
    """
    Represents an entity in an Entity Component architecture.
    An entity has an ID and a collection of components.
    """
    def __init__(self, entity_id):
        self.entity_id = entity_id
        self.components = {}

    def add_component(self, component):
        self.components[type(component)] = component

    def get_component(self, component_type):
        return self.components.get(component_type)


class EntityComponentManager:
    """Manages the entities in the simulation."""
    def __init__(self):
        self.next_entity_id = 0
        self.entities = {}

    def create_entity(self):
        entity_id = self.next_entity_id
        self.next_entity_id += 1
        entity = Entity(entity_id)
        self.entities[entity_id] = entity
        return entity

    def add_component_to_entity(self, entity, component):
        entity.add_component(component)

    def get_entities_with_component(self, component_type):
        return [entity for entity in self.entities.values()
                if entity.get_component(component_type) is not None]
