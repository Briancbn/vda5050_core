# Copyright (C) 2026 ROS-Industrial Consortium Asia Pacific
# Advanced Remanufacturing and Technology Centre
# A*STAR Research Entities (Co. Registration No. 199702110H)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import pytest
from vda5050_core.master import AGV, VDA5050Master


def test_master_imports():
    assert AGV is not None
    assert VDA5050Master is not None


@pytest.fixture
def mock_mqtt_client(mock_mqtt_client):
    connected = False

    def connect():
        nonlocal connected
        connected = True

    def disconnect():
        nonlocal connected
        connected = False

    mock_mqtt_client.connected.side_effect = lambda: connected
    mock_mqtt_client.connect.side_effect = connect
    mock_mqtt_client.disconnect.side_effect = disconnect

    return mock_mqtt_client


def test_master_mock_transport_connect(mock_mqtt_client):
    master = VDA5050Master.make(mock_mqtt_client)
    assert master.is_connected() is False
    assert mock_mqtt_client.connected.call_count == 1

    master.connect()
    assert mock_mqtt_client.connect.call_count == 1
    assert master.is_connected() is True

    master.disconnect()
    assert mock_mqtt_client.disconnect.call_count == 1
    assert master.is_connected() is False
