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
    """A base component class that can be added to an entity."""

    def __init__(self):
        self.entity = None

    def set_entity(self, entity):
        self.entity = entity

    def update(self, delta_time, sim_state, entity, ecm):
        pass

    def get_state(self):
        return None


class ComponentContainer(Component):
    """
    A component that stores other components.

    An example of a component container is a weapon that stores damage, range, and firing rate components.
    class Weapon(ComponentContainer):
        def __init__(self):
            super().__init__()
            self.add_component(Damage(10))
            self.add_component(Range(100))
            self.add_component(FiringRate(5))

        def update(self, delta_time, sim_state, entity, ecm):
            super().update(delta_time, sim_state, entity, ecm)
            # additional weapon behavior
            pass
    """

    def __init__(self):
        super().__init__()
        self.components = {}

    def add_component(self, component):
        component_type = type(component)
        if component_type not in self.components:
            self.components[component_type] = []
        self.components[component_type].append(component)
        component.set_entity(self)

    def get_component(self, component_type):
        return self.components.get(component_type, [])

    def update(self, delta_time, sim_state, entity, ecm):
        super().update(delta_time, sim_state, entity, ecm)
        for component in self.components.values():
            component.update(delta_time, sim_state, entity, ecm)

    def get_state(self):
        state = {}
        for component_type, component_list in self.components.items():
            state[component_type.__name__] = [c.get_state() for c in component_list]
        return state


class Name(Component):
    """A component that stores a name."""

    def __init__(self, name):
        super().__init__()
        self.name = name

    def get_state(self):
        return self.name


class Type(Component):
    """
    A component that stores a type.
    Useful for distinguishing between different types of entities.
    """

    def __init__(self, type):
        super().__init__()
        self.type = type

    def get_state(self):
        return self.type


class Pose(Component):
    """A component that stores a position and rotation."""

    def __init__(self, position, rotation):
        self.position = position
        self.rotation = rotation

    def get_state(self):
        return self.position + self.rotation


class Velocity(Component):
    """A component that stores a linear and angular velocity."""

    def __init__(self, linear, angular):
        super().__init__()
        self.linear = linear
        self.angular = angular

    def get_state(self):
        return self.linear + self.angular


class RoadNetwork(Component):
    """A component that stores a road network."""

    def __init__(self, road_network):
        super().__init__()
        self.road_network = road_network

    def get_state(self):
        # TODO: Evaluate if this is the best way to store the maliput road network.
        return None
