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

import os

from maliput_sim.core.components import Pose, Velocity
from maliput_sim.simulation import Simulation, SimulationConfig, AgentInitialState

import maliput.plugin


# Controller to be executed for the agent during the step
def agent_controller(duration, sim_state, entity, ecm):
    """Agent controller."""
    pose = entity.get_components(Pose)[0]
    velocity = entity.get_components(Velocity)[0]

    # Move the agent forward in X direction using the current velocity.
    pose._position[0] += duration * velocity.get_linear_vel()[0]


def main():
    """Main entry point."""

    # Create road network model
    env = os.getenv("MALIPUT_MALIDRIVE_RESOURCE_ROOT")
    rn_configuration = {"opendrive_file": env +
                        "/resources/odr/TShapeRoad.xodr" if env is not None else ""}
    maliput_road_network = maliput.plugin.create_road_network(
        "maliput_malidrive", rn_configuration)

    # Sim config
    sim_config = SimulationConfig(real_time_factor=1., time_step=0.01)

    # Create simulation instance
    sim = Simulation(maliput_road_network, sim_config)

    # Create an agent.
    # AgentState: name, position, rotation, linear_vel, angular_vel
    initial_agent_state = AgentInitialState(
        "agent_1", [0., 0., 0.], [0., 0., 0., 1], 1., 0.)
    sim.add_agent(initial_agent_state, agent_controller)

    # Run Simulation for 1 second : 100 steps
    agent_position = sim.get_ecm().get_entities_with_component(Pose)[
        0].get_components(Pose)[0].get_position()
    for i in range(100):
        sim.step()
        print("Sim time: ", sim.get_sim_state().sim_time)
        print("Agent Position: ", agent_position)


if __name__ == "__main__":
    main()
