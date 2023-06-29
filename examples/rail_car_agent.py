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

from maliput_sim.core.sim import SimulationConfig, AgentInitialState
from maliput_sim.simulation import Simulation
from maliput_sim.viz.plotly import Plotly

import maliput.plugin
import maliput.utility

from controllers import RailCarController

def main():
    """Main entry point."""

    # Create road network model
    env = os.getenv("MALIPUT_MALIDRIVE_RESOURCE_ROOT").split(":")
    env = env[0]
    print(env)
    rn_configuration = {"opendrive_file": env +
                        "/resources/odr/FlatTown01.xodr" if env is not None else "",
                        "road_geometry_id": "FlatTown01_RoadNetwork"}
    maliput_road_network = maliput.plugin.create_road_network(
        "maliput_malidrive", rn_configuration)

    # Sim config
    sim_config = SimulationConfig(real_time_factor=0., time_step=0.05)

    # Create simulation instance
    sim = Simulation(maliput_road_network, sim_config)

    # Add agents.
    # AgentState: name, position, rotation, linear_vel, angular_vel

    initial_agent_state_1 = AgentInitialState(
        "agent_1", [20., -1., 0.], [0., 0., 0., 1], 10., 0.)
    rail_car_1 = RailCarController()
    sim.add_agent(initial_agent_state_1, rail_car_1.step, rail_car_1.get_state)

    initial_agent_state_2 = AgentInitialState(
        "agent_2", [90., -40., 0.], [0., 0., 0., 1], 20., 0.)
    rail_car_2 = RailCarController()
    sim.add_agent(initial_agent_state_2, rail_car_2.step, rail_car_2.get_state)

    initial_agent_state_3 = AgentInitialState(
        "agent_3", [240., -60., 0.], [0., 0., 0., 1], 30., 0.)
    rail_car_3 = RailCarController()
    sim.add_agent(initial_agent_state_3, rail_car_3.step, rail_car_3.get_state)


    # Run Simulation for 30 seconds
    print("Running...")
    sim.step_for(30)

    # Use plotly to visualize the simulation
    viz = Plotly(sim)
    viz.show()

if __name__ == "__main__":
    main()
