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
from typing import Any, Callable, Dict

from maliput_sim.core.components import Component
from maliput_sim.core.ecm import Entity, EntityComponentManager


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
    ecm_state: dict


class Behavior(Component):
    """A component that stores a behavior.
    The behavior is callable with the same interface as update() and implements
    their own logic after this component's parent update() finishes."""

    def __init__(self,
                 behavior: Callable[[float, SimulationState, Entity, EntityComponentManager], None],
                 get_state: Callable[[], Dict[str, Any]] = lambda: {}):
        """Constructs a behavior component.
        Args:
            behavior: The behavior to be called during update().
            get_state: A function that returns the state of the behavior.

        TODO(francocipollone): Use typing.TypeAlias when moving to Python 3.10 for better readability.
        """
        super().__init__()
        self._behavior = behavior
        self._get_state = get_state

    def update(self, delta_time: float,
               sim_state: SimulationState,
               entity: Entity,
               ecm: EntityComponentManager):
        """
        Calls the behavior function.

        Args:
            delta_time: The amount of time that has passed since the last update.
            sim_state: The state of the simulation.
            entity: The entity that this component belongs to.
            ecm: The entity component manager.
        """
        self._behavior(delta_time, sim_state, entity, ecm)

    def get_state(self):
        """Returns the state of the behavior."""
        return self._get_state()
