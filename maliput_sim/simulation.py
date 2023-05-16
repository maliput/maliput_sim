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

from dataclasses import dataclass

from maliput_sim.core.components import *
from maliput_sim.core.ecm import *


@dataclass
class SimulationConfig:
    """The configuration of the simulation."""
    real_time_factor: float = 1.
    time_step: float = 0.01


@dataclass
class AgentInitialState:
    """The initial state of an agent in the simulation."""
    name: str
    position: list
    rotation: list
    linear_vel: float
    angular_vel: float


@dataclass
class SimulationState:
    """The state of the simulation."""
    sim_time: float


class Simulation:
    def __init__(self, road_network, sim_config):
        """Initializes a simulation.

        Args:
            road_network: The road network to simulate on.
            sim_config: The simulation configuration.

        """
        self._sim_state = SimulationState(0.0)
        self._entity_component_manager = EntityComponentManager()

        # Add the road network to the simulation.
        rn_entity = self._entity_component_manager.create_entity()
        rn_entity.add_component(Name(road_network.road_geometry().id()))
        rn_entity.add_component(Type("road_network"))
        rn_entity.add_component(RoadNetwork(road_network))

        # Saves the simulation configuration.
        self._sim_config = sim_config

    def add_agent(self, initial_state: AgentInitialState, controller):
        """Adds an agent to the simulation.

        Args:
            agent: The agent to add.

        """
        agent_entity = self._entity_component_manager.create_entity()
        agent_entity.add_component(Name(initial_state.name))
        agent_entity.add_component(Type("agent"))
        agent_entity.add_component(
            Pose(initial_state.position, initial_state.rotation))
        agent_entity.add_component(
            Velocity(initial_state.linear_vel, initial_state.angular_vel))
        agent_entity.add_component(Behavior(controller))

    def step(self):
        """Advances the simulation by one step.

        Returns:
            The new simulation state.
        """
        self._entity_component_manager.update(
            self._sim_config.time_step, self._sim_state)
        self._sim_state.sim_time += self._sim_config.time_step

    def step_n(self, n):
        """Advances the simulation by the specified number of steps.

        Args:
            n: The number of steps to advance the simulation by.

        Returns:
            The new simulation state.
        """
        for _ in range(n):
            self.step()

    def step_for(self, duration):
        """Advances the simulation by the specified duration.

        Args:
            duration: The duration to advance the simulation by in seconds.

        Returns:
            The new simulation state.
        """

        number_of_steps = int(duration / self._sim_config.time_step)
        if number_of_steps is 0:
            number_of_steps = 1

        self.step_n(number_of_steps)

    def get_sim_state(self):
        """Returns the current simulation state.

        Returns:
            The current simulation state.
        """
        # TODO: Implement this.
        return self._sim_state

    def get_ecm(self):
        """Returns the entity component manager.

        Returns:
            The entity component manager.
        """
        return self._entity_component_manager
