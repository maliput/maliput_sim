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

from maliput_sim.simulation import Simulation
from maliput_sim.utils.obj import generate_obj_file_from_road_network
from maliput_sim.viz.base import Visualizer

import pywavefront
import plotly.graph_objects as go


def get_vertices_and_faces_from_obj(obj_filepath):
    """Returns the vertices and faces from an obj file."""
    obj_data = pywavefront.Wavefront(obj_filepath, collect_faces=True)
    vertices = obj_data.vertices
    faces = obj_data.mesh_list[0].faces
    return vertices, faces


def get_vertices_and_faces_from_road_network(road_network):
    """Returns the vertices and faces from a road network."""
    obj_filepath = generate_obj_file_from_road_network(road_network)
    return get_vertices_and_faces_from_obj(obj_filepath)


class Plotly(Visualizer):
    _TRACE_WIDTH = 5
    _ROAD_NETWORK_COLOR = 'lightblue'

    """A Plotly visualization."""
    def __init__(self, simulation: Simulation):
        self._simulation = simulation

    def show(self):
        """
        Display the visualization.
        Ideally this method should be called after the simulation is complete.
        """

        self._fig = go.Figure()

        road_network = self._simulation.get_road_network()

        vertices, faces = get_vertices_and_faces_from_road_network(road_network)

        # Create a Mesh3d trace
        mesh = go.Mesh3d(
            x=[v[0] for v in vertices],
            y=[v[1] for v in vertices],
            z=[v[2] for v in vertices],
            i=[f[0] for f in faces],
            j=[f[1] for f in faces],
            k=[f[2] for f in faces],
            color=self._ROAD_NETWORK_COLOR,
            name=road_network.road_geometry().id().string(),
        )

        self._fig.add_trace(mesh)
        sim_states = self._simulation.get_sim_states()

        self._agent_poses = {}
        for sim_state in sim_states:
            if (not sim_state.ecm_state):
                continue
            entities = sim_state.ecm_state['entities']
            agent_entities = (
                entity for entity in entities if 'agent' in entity.get('components', {}).get('Type', []))

            # Add agent information
            for sim_state_entity in agent_entities:
                components = sim_state_entity['components']
                name = components['Name'][0]
                pose = components['Pose'][0]

                if name not in self._agent_poses:
                    self._agent_poses[name] = []
                self._agent_poses[name].append(pose.copy())

        # Add traces
        for agent_name, poses in self._agent_poses.items():
            scatter = go.Scatter3d(
                x=[p[0] for p in poses],
                y=[p[1] for p in poses],
                z=[p[2] for p in poses],
                name=agent_name,
                mode='lines',
                line=dict(
                    width=self._TRACE_WIDTH
                )
            )
            self._fig.add_trace(scatter)
        self._fig.show()
