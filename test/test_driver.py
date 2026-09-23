#
# Copyright (C) 2014-2015  UAVCAN Development Team  <uavcan.org>
#
# This software is distributed under the terms of the MIT License.
#
# Author: Ben Dyer <ben_dyer@mac.com>
#         Pavel Kirienko <pavel.kirienko@zubax.com>
#

import unittest
from unittest import mock
from dronecan import driver
from dronecan.driver.mavcan import MAVCAN


class TestMAVCANDetection(unittest.TestCase):
    @mock.patch('dronecan.driver.mavcan.mavutil.mavlink_connection')
    def test_detect_mavlink_baud_returns_fallback_baud(self, mavlink_connection):
        connections = []

        def open_connection(_device_name, **kwargs):
            connection = mock.Mock()
            connection.recv_match.return_value = object() if kwargs['baud'] == 115200 else None
            connections.append(connection)
            return connection

        mavlink_connection.side_effect = open_connection

        self.assertEqual(115200, MAVCAN.detect_mavlink_baud('COM1', 921600))
        self.assertEqual([921600, 115200], [call.kwargs['baud'] for call in mavlink_connection.call_args_list])
        self.assertTrue(all(connection.close.called for connection in connections))

    @mock.patch('dronecan.driver.MAVCAN')
    @mock.patch('dronecan.driver._detect_mavlink_baud', return_value=115200)
    def test_make_driver_uses_detected_mavlink_baud(self, detect_mavlink_baud, mavcan):
        driver.make_driver('COM1')

        detect_mavlink_baud.assert_called_once_with('COM1')
        mavcan.assert_called_once_with('COM1', baudrate=115200)


if __name__ == '__main__':
    unittest.main()
