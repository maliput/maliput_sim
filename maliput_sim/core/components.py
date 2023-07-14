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

class Entity: pass
class EntityComponentManager: pass
class SimulationState: pass

class Component:
    """A base component class that can be added to an entity."""

    def __init__(self):
        self.entity = None

    def set_entity(self, entity: 'Entity'):
        """
        Set the entity that this component belongs to.

        Args:
            entity: The entity that this component belongs to.

        """
        self.entity = entity

    def update(self, delta_time: float, sim_state: 'SimulationState', entity: 'Entity', ecm: 'EntityComponentManager'):
        """
        Update the component.

        Args:
            delta_time: The amount of time that has passed since the last update.
            sim_state: The current simulation state.
            entity: The entity that this component belongs to.
            ecm: The entity component manager.
        """
        pass

    def get_state(self):
        """
        Get the current state of the component.
        """
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

    def add_component(self, component: Component):
        """
        Add a component to the container.

        Args:
            component: The component to add to the container.
        """
        component_type = type(component)
        if component_type not in self.components:
            self.components[component_type] = []
        self.components[component_type].append(component)
        component.set_entity(self)

    def get_component(self, component_type: type):
        """
        Get the list of components of the specified type.

        Args:
            component_type: The type of component to get.

        Returns:
            A list of components of the specified type.
        """
        return self.components.get(component_type, [])

    def update(self, delta_time: float, sim_state: 'SimulationState', entity: 'Entity', ecm: 'EntityComponentManager'):
        """
        Updates all the components in the container.

        Args:
            delta_time: The amount of time that has passed since the last update.
            sim_state: The current simulation state.
            entity: The entity that this component belongs to.
            ecm: The entity component manager.
        """
        super().update(delta_time, sim_state, entity, ecm)
        for component in self.components.values():
            component.update(delta_time, sim_state, entity, ecm)

    def get_state(self):
        """
        Get the current state of the component container.
        Each component in the container will have its own state.
        The state of the component container is a dictionary of the states of each component.
        The keys of the dictionary are the type names of the components. And the value of each key is a list of the
        states of the components of that type.
        """
        state = {}
        for component_type, component_list in self.components.items():
            state[component_type.__name__] = [c.get_state() for c in component_list]
        return state


class Name(Component):
    """A component that stores a name."""

    def __init__(self, name: str):
        super().__init__()
        self.name = name

    def get_state(self) -> str:
        return self.name


class Type(Component):
    """
    A component that stores a type.
    Useful for distinguishing between different types of entities.
    """

    def __init__(self, type: str):
        super().__init__()
        self.type = type

    def get_state(self) -> str:
        return self.type


class Pose(Component):
    """A component that stores a position and rotation."""

    def __init__(self, position: list, rotation: list):
        """
        Initialize the pose component.

        Args:
            position: The position of the entity: [x y z].
            rotation: The rotation of the entity in quaternion format: [x y z w].
        """
        self.position = position
        self.rotation = rotation

    def get_state(self) -> list:
        return self.position + self.rotation


class Velocity(Component):
    """A component that stores a linear and angular velocity."""

    def __init__(self, linear: list, angular: list):
        """
        Initialize the velocity component.

        Args:
            linear: The linear velocity of the entity: [x y z].
            angular: The angular velocity of the entity around each axis: [x y z].
        """
        super().__init__()
        self.linear = linear
        self.angular = angular

    def get_state(self):
        return self.linear + self.angular


class RoadNetwork(Component):
    """A component that stores a road network."""

    def __init__(self, road_network):
        """
        Initialize the road network component.

        Args:
            road_network: The maliput road network.
        """
        super().__init__()
        self.road_network = road_network

    def get_state(self):
        # TODO: Evaluate if this is the best way to store the maliput road network.
        return None
