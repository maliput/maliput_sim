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
from tempfile import gettempdir

from maliput.api import RoadNetwork
import maliput.utility

def generate_obj_file_from_road_network(road_network: RoadNetwork, output_directory=None):
    """
    Generates an obj file from a road network.
    Args:
        road_network: A maliput road network.
        output_directory: The output directory where the obj file will be saved.(optional)
    """
    out_dir = output_directory if output_directory else gettempdir() + "/maliput_sim"
    # Check if the directory exists
    if not os.path.exists(out_dir):
        # If it doesn't exist, create it
        os.makedirs(out_dir)

    # Create obj
    obj_features = maliput.utility.ObjFeatures()
    obj_features.draw_arrows = False
    obj_features.draw_stripes = True
    obj_features.draw_elevation_bounds = False
    obj_features.draw_branch_points = False
    obj_features.draw_lane_haze = False
    maliput.utility.GenerateObjFile(
        road_network, out_dir, "road_network", obj_features)
    return out_dir + "/road_network.obj"


def get_obj_description_from_road_network(road_network: RoadNetwork):
    """Returns the obj description from a road network."""
    obj_filepath = generate_obj_file_from_road_network(road_network)
    with open(obj_filepath, "r") as f:
        obj_description = f.read()
    return obj_description
