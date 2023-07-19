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

import unittest
from unittest.mock import MagicMock

from maliput_sim.simulation import Simulation
from maliput_sim.export.base import Exporter


class ExporterMock(Exporter):
    """
    Inherits from Exporter and overrides the dump method.
    This allows us to test the methods of the Exporter class without relying on the actual implementation.
    """
    _dump_method_called = False

    def dump(self, export_agent_param=True, export_road_network_param=True,
             init_timestamp=0.0, end_timestamp=0.0, export_signals=[]):
        self._dump_method_called = True


class TestExporterMethods(unittest.TestCase):

    def setUp(self):
        # Mocking the Simulation instance
        self.simulation = MagicMock(spec=Simulation)
        self.dut = ExporterMock(self.simulation)

    def test_init(self):
        # Check if the exporter is initialized with the correct Simulation instance
        self.assertEqual(self.dut._simulation, self.simulation)

    def test_dump(self):
        # Call the dump() method with some parameters
        export_agent_param = True
        export_road_network_param = False
        init_timestamp = 1.0
        end_timestamp = 10.0
        export_signals = ["signal1", "signal2"]

        self.dut.dump(export_agent_param=export_agent_param,
                      export_road_network_param=export_road_network_param,
                      init_timestamp=init_timestamp,
                      end_timestamp=end_timestamp,
                      export_signals=export_signals)

        # Check if the dump method was called
        self.assertTrue(self.dut._dump_method_called)
