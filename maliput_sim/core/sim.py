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
from typing import Callable

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


class Behavior(Component):
    """A component that stores a behavior.
    The behavior is callable with the same interface as update() and implements
    their own logic after this component's parent update() finishes."""

    def __init__(self, behavior: Callable[[float, SimulationState, Entity, EntityComponentManager], None]):
        """Constructs a behavior component.
        Args: behavior: The behavior to be called during update().

        TODO(francocipollone): Use typing.TypeAlias when moving to Python 3.10 for better readability.
        """
        super().__init__()
        self.behavior = behavior

    def update(self, delta_time, sim_state, entity, ecm):
        """Calls the behavior after the parent update() finishes."""
        super().update(delta_time, sim_state, entity, ecm)
        self.behavior(delta_time, sim_state, entity, ecm)
