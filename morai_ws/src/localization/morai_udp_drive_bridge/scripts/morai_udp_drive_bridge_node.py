#!/usr/bin/env python3
"""MORAI Competition Vehicle Status RX and Ego Ctrl Cmd TX bridge."""

from __future__ import annotations

import math
import socket
import threading
import time
from typing import Optional

import rospy
from common.msg import CompetitionVehicleStatus
from geometry_msgs.msg import Quaternion, Vector3
from morai_msgs.msg import CtrlCmd
from std_msgs.msg import Float64

from morai_udp_drive_bridge.protocol import (
    COMPETITION_STATUS_PACKET_SIZE,
    ProtocolError,
    build_ego_ctrl_cmd,
    parse_competition_vehicle_status,
)


def quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> Quaternion:
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return Quaternion(
        x=sr * cp * cy - cr * sp * sy,
        y=cr * sp * cy + sr * cp * sy,
        z=cr * cp * sy - sr * sp * cy,
        w=cr * cp * cy + sr * sp * sy,
    )


class MoraiUdpDriveBridge:
    def __init__(self) -> None:
        rospy.init_node("morai_udp_drive_bridge", anonymous=False)

        self.status_bind_ip = rospy.get_param("~status_bind_ip", "0.0.0.0")
        self.status_port = int(rospy.get_param("~status_port", 9099))
        self.status_expected_source_port = int(
            rospy.get_param("~status_expected_source_port", 9088)
        )
        self.status_expected_source_ip = rospy.get_param("~status_expected_source_ip", "")
        self.status_topic = rospy.get_param(
            "~status_topic", "/competition/vehicle_status"
        )
        self.speed_topic = rospy.get_param(
            "~speed_topic", "/competition/vehicle_speed_mps"
        )
        self.status_frame_id = rospy.get_param("~status_frame_id", "base_link")
        # Read once at startup: dry-run output cannot be enabled at runtime.
        self.control_output_enabled = bool(rospy.get_param("~control_output_enabled", True))
        self.control_remote_ip = rospy.get_param("~control_remote_ip", "192.168.0.147")
        self.control_remote_port = int(rospy.get_param("~control_remote_port", 9093))
        self.control_bind_ip = rospy.get_param("~control_bind_ip", "0.0.0.0")
        self.control_bind_port = int(rospy.get_param("~control_bind_port", 0))
        self.command_topic = rospy.get_param("~command_topic", "/ctrl_cmd")
        self.send_rate_hz = float(rospy.get_param("~send_rate_hz", 20.0))
        self.command_timeout_sec = float(rospy.get_param("~command_timeout_sec", 0.5))
        self.max_wheel_angle_rad = float(
            rospy.get_param("~max_wheel_angle_rad", math.radians(40.0))
        )
        self.status_use_packet_time = bool(rospy.get_param("~status_use_packet_time", False))
        self.longl_cmd_type = int(rospy.get_param("~longl_cmd_type", 1))

        self.last_command: Optional[CtrlCmd] = None
        self.last_command_time = 0.0
        self.stop_event = threading.Event()
        self.send_socket = None
        self.send_timer = None

        self.status_pub = rospy.Publisher(
            self.status_topic, CompetitionVehicleStatus, queue_size=20
        )
        self.speed_pub = rospy.Publisher(self.speed_topic, Float64, queue_size=20)
        rospy.Subscriber(self.command_topic, CtrlCmd, self.command_callback, queue_size=20)

        if self.control_output_enabled:
            self.send_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            if self.control_bind_port > 0:
                self.send_socket.bind((self.control_bind_ip, self.control_bind_port))
            self.send_timer = rospy.Timer(
                rospy.Duration(1.0 / max(self.send_rate_hz, 1.0)), self.send_timer_callback
            )

        self.status_thread = threading.Thread(
            target=self.status_receive_loop,
            name="morai-competition-status-receiver",
            daemon=True,
        )
        self.status_thread.start()
        rospy.on_shutdown(self.shutdown)

        rospy.loginfo(
            "MORAI UDP drive bridge: status bind=%s:%d topic=%s, control remote=%s:%d, output enabled=%s",
            self.status_bind_ip,
            self.status_port,
            self.status_topic,
            self.control_remote_ip,
            self.control_remote_port,
            self.control_output_enabled,
        )

    def status_receive_loop(self) -> None:
        receive_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        receive_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        receive_socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024 * 1024)
        try:
            receive_socket.bind((self.status_bind_ip, self.status_port))
        except OSError as exc:
            rospy.logfatal(
                "CompetitionVehicleStatus UDP bind failed %s:%d: %s",
                self.status_bind_ip,
                self.status_port,
                exc,
            )
            receive_socket.close()
            return
        receive_socket.settimeout(0.5)
        rospy.loginfo(
            "CompetitionVehicleStatus UDP bind %s:%d expected=%d bytes source=%s:%d",
            self.status_bind_ip,
            self.status_port,
            COMPETITION_STATUS_PACKET_SIZE,
            self.status_expected_source_ip or "*",
            self.status_expected_source_port,
        )

        try:
            while not rospy.is_shutdown() and not self.stop_event.is_set():
                try:
                    packet, sender = receive_socket.recvfrom(65535)
                except socket.timeout:
                    continue
                except OSError as exc:
                    if not self.stop_event.is_set():
                        rospy.logerr_throttle(
                            5.0, "CompetitionVehicleStatus UDP receive error: %s", exc
                        )
                    break

                if self.status_expected_source_ip and sender[0] != self.status_expected_source_ip:
                    rospy.logwarn_throttle(
                        5.0, "Competition status rejected from unexpected IP %s", sender[0]
                    )
                    continue
                if self.status_expected_source_port > 0 and sender[1] != self.status_expected_source_port:
                    rospy.logwarn_throttle(
                        5.0,
                        "Competition status rejected from unexpected source port %d",
                        sender[1],
                    )
                    continue

                try:
                    measurement = parse_competition_vehicle_status(packet)
                except (ProtocolError, ValueError) as exc:
                    rospy.logwarn_throttle(
                        5.0,
                        "CompetitionVehicleStatus rejected sender=%s:%d len=%d error=%s",
                        sender[0],
                        sender[1],
                        len(packet),
                        exc,
                    )
                    continue

                self.publish_status(measurement)
        finally:
            receive_socket.close()

    def publish_status(self, measurement) -> None:
        message = CompetitionVehicleStatus()
        if self.status_use_packet_time and measurement.sec > 0:
            message.header.stamp = rospy.Time(measurement.sec, measurement.nsec)
        else:
            message.header.stamp = rospy.Time.now()
        message.header.frame_id = self.status_frame_id
        message.control_mode = measurement.ctrl_mode
        message.gear = measurement.gear
        message.signed_velocity_kph = measurement.signed_velocity_kmh
        message.map_data_id = measurement.map_data_id
        message.orientation = quaternion_from_rpy(
            math.radians(measurement.roll_deg),
            math.radians(measurement.pitch_deg),
            math.radians(measurement.heading_deg),
        )
        message.velocity_x_mps = measurement.velocity_x_kmh / 3.6
        message.angular_velocity = Vector3(
            math.radians(measurement.angular_velocity_x_deg_s),
            math.radians(measurement.angular_velocity_y_deg_s),
            math.radians(measurement.angular_velocity_z_deg_s),
        )
        message.accel = measurement.accel_pedal
        message.brake = measurement.brake_pedal
        message.steer_rad = math.radians(measurement.steer_deg)
        message.packet_size = measurement.packet_length
        self.status_pub.publish(message)
        self.speed_pub.publish(Float64(data=message.velocity_x_mps))
        rospy.loginfo_throttle(
            2.0,
            "Competition status vx=%.2fm/s rpy=(%.1f,%.1f,%.1f)deg link=%s",
            measurement.velocity_x_kmh / 3.6,
            measurement.roll_deg,
            measurement.pitch_deg,
            measurement.heading_deg,
            measurement.link_id,
        )

    def command_callback(self, message: CtrlCmd) -> None:
        self.last_command = message
        self.last_command_time = time.monotonic()

    def send_timer_callback(self, _event) -> None:
        if not self.control_output_enabled or self.send_socket is None or self.stop_event.is_set():
            return

        message = self.last_command
        is_fresh = message is not None and time.monotonic() - self.last_command_time <= self.command_timeout_sec

        if not is_fresh:
            packet = build_ego_ctrl_cmd(
                cmd_type=self.longl_cmd_type, velocity_kmh=0.0, brake=1.0
            )
        else:
            cmd_type = int(getattr(message, "longlCmdType", self.longl_cmd_type))
            steering_rad = float(getattr(message, "steering", 0.0))
            steer_normalized = steering_rad / max(self.max_wheel_angle_rad, 1e-6)
            velocity_kmh = (
                max(0.0, float(getattr(message, "velocity", 0.0)) * 3.6)
                if cmd_type == 2
                else 0.0
            )
            packet = build_ego_ctrl_cmd(
                cmd_type=cmd_type,
                velocity_kmh=velocity_kmh,
                acceleration_mps2=float(getattr(message, "acceleration", 0.0)),
                accel=float(getattr(message, "accel", 0.0)),
                brake=float(getattr(message, "brake", 0.0)),
                steer_normalized=steer_normalized,
                ctrl_mode=2,
                gear=4,
            )

        try:
            self.send_socket.sendto(packet, (self.control_remote_ip, self.control_remote_port))
        except OSError as exc:
            rospy.logerr_throttle(5.0, "EgoCtrlCmd UDP 송신 오류: %s", exc)

    def shutdown(self) -> None:
        self.stop_event.set()
        if self.send_timer is not None:
            self.send_timer.shutdown()
        if not self.control_output_enabled or self.send_socket is None:
            return

        try:
            stop_packet = build_ego_ctrl_cmd(
                cmd_type=self.longl_cmd_type, velocity_kmh=0.0, brake=1.0
            )
            self.send_socket.sendto(stop_packet, (self.control_remote_ip, self.control_remote_port))
        except OSError:
            pass
        finally:
            self.send_socket.close()


if __name__ == "__main__":
    try:
        MoraiUdpDriveBridge()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
