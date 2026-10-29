import struct
import unittest

from morai_udp_drive_bridge.protocol import (
    COMPETITION_STATUS_PACKET_SIZE,
    EGO_CTRL_CMD_PACKET_SIZE,
    LEGACY_EGO_STATUS_CORE_PACKET_SIZE,
    LEGACY_EGO_STATUS_PACKET_SIZE,
    ProtocolError,
    build_ego_ctrl_cmd,
    parse_competition_vehicle_status,
)


class CompetitionProtocolTest(unittest.TestCase):
    def test_control_packet(self):
        packet = build_ego_ctrl_cmd(
            cmd_type=2,
            velocity_kmh=7.2,
            steer_normalized=0.5,
        )
        self.assertEqual(len(packet), EGO_CTRL_CMD_PACKET_SIZE)
        self.assertEqual(packet[:14], b"#MoraiCtrlCmd$")

    def test_status_packet(self):
        floats = [float(index) for index in range(16)]
        packet = struct.pack(
            "<11s i 3i i i b b f i 16f 38s 2s",
            b"#MoraiInfo$",
            70,
            0,
            0,
            0,
            10,
            20,
            2,
            4,
            7.2,
            10000,
            *floats,
            b"LINK_1",
            b"\r\n",
        )
        self.assertEqual(len(packet), COMPETITION_STATUS_PACKET_SIZE)
        status = parse_competition_vehicle_status(packet)
        self.assertEqual(status.heading_deg, 10.0)
        self.assertEqual(status.velocity_x_kmh, 11.0)
        self.assertEqual(status.steer_deg, 15.0)
        self.assertEqual(status.link_id, "LINK_1")

    def test_legacy_ego_status_is_explicitly_rejected(self):
        for packet_size in (
            LEGACY_EGO_STATUS_PACKET_SIZE,
            LEGACY_EGO_STATUS_CORE_PACKET_SIZE,
        ):
            with self.subTest(packet_size=packet_size):
                with self.assertRaisesRegex(ProtocolError, "forbidden"):
                    parse_competition_vehicle_status(bytes(packet_size))


if __name__ == "__main__":
    unittest.main()
