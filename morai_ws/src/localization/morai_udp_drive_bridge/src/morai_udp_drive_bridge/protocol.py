"""MORAI competition vehicle UDP protocol.

The competition publisher is the restricted subset of EgoVehicleStatus
described by the 2026 KATRI rules.  Its wire layout removes position, Y/Z
velocity and linear acceleration from the legacy packed status.  Keeping a
separate 149-byte parser is intentional: a forbidden 181-byte Ego status
packet must never be accepted as competition input.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import struct


COMPETITION_STATUS_FORMAT = "<11s i 3i i i b b f i 16f 38s 2s"
COMPETITION_STATUS_PACKET_SIZE = struct.calcsize(COMPETITION_STATUS_FORMAT)
# Full EgoVehicleStatus has 24 body floats, a 38-byte link id and 12 tyre
# floats.  Some older local code stopped at link_id and treated 181 bytes as
# the whole packet, so reject that truncated legacy shape explicitly too.
LEGACY_EGO_STATUS_PACKET_SIZE = struct.calcsize(
    "<11s i 3i i i b b f i 24f 38s 12f 2s"
)
LEGACY_EGO_STATUS_CORE_PACKET_SIZE = struct.calcsize(
    "<11s i 3i i i b b f i 24f 38s 2s"
)
EGO_CTRL_CMD_FORMAT = "<14s i 3i 3b 5f 2s"
EGO_CTRL_CMD_PACKET_SIZE = struct.calcsize(EGO_CTRL_CMD_FORMAT)


class ProtocolError(ValueError):
    """Raised when a MORAI competition UDP packet violates the contract."""


@dataclass(frozen=True)
class CompetitionVehicleStatusMeasurement:
    sec: int
    nsec: int
    ctrl_mode: int
    gear: int
    signed_velocity_kmh: float
    map_data_id: int
    accel_pedal: float
    brake_pedal: float
    size_x_m: float
    size_y_m: float
    size_z_m: float
    overhang_m: float
    wheelbase_m: float
    rear_overhang_m: float
    roll_deg: float
    pitch_deg: float
    heading_deg: float
    velocity_x_kmh: float
    angular_velocity_x_deg_s: float
    angular_velocity_y_deg_s: float
    angular_velocity_z_deg_s: float
    steer_deg: float
    link_id: str
    packet_length: int


def _finite(value: float, name: str) -> float:
    value = float(value)
    if not math.isfinite(value):
        raise ProtocolError(f"CompetitionVehicleStatus {name} is not finite: {value}")
    return value


def parse_competition_vehicle_status(packet: bytes) -> CompetitionVehicleStatusMeasurement:
    if len(packet) in (
        LEGACY_EGO_STATUS_PACKET_SIZE,
        LEGACY_EGO_STATUS_CORE_PACKET_SIZE,
    ):
        raise ProtocolError(
            "legacy EgoVehicleStatus is forbidden by the competition rules"
        )
    if len(packet) != COMPETITION_STATUS_PACKET_SIZE:
        raise ProtocolError(
            "CompetitionVehicleStatus packet length error: "
            f"expected={COMPETITION_STATUS_PACKET_SIZE}, actual={len(packet)}"
        )

    values = struct.unpack(COMPETITION_STATUS_FORMAT, packet)
    header = values[0]
    if header not in (b"#MoraiInfo$", b"#MoraiStatu"):
        raise ProtocolError(f"unexpected CompetitionVehicleStatus header: {header!r}")
    if values[-1] != b"\r\n":
        raise ProtocolError("CompetitionVehicleStatus footer is not CRLF")

    sec = int(values[5])
    nsec = int(values[6])
    if sec < 0 or not 0 <= nsec < 1_000_000_000:
        raise ProtocolError(f"CompetitionVehicleStatus timestamp error: sec={sec}, nsec={nsec}")

    floats = [_finite(value, "float") for value in values[11:27]]
    link_id = values[27].split(b"\x00", 1)[0].decode("utf-8", errors="replace")
    return CompetitionVehicleStatusMeasurement(
        sec=sec,
        nsec=nsec,
        ctrl_mode=int(values[7]),
        gear=int(values[8]),
        signed_velocity_kmh=_finite(values[9], "signed_velocity_kmh"),
        map_data_id=int(values[10]),
        accel_pedal=floats[0],
        brake_pedal=floats[1],
        size_x_m=floats[2],
        size_y_m=floats[3],
        size_z_m=floats[4],
        overhang_m=floats[5],
        wheelbase_m=floats[6],
        rear_overhang_m=floats[7],
        roll_deg=floats[8],
        pitch_deg=floats[9],
        heading_deg=floats[10],
        velocity_x_kmh=floats[11],
        angular_velocity_x_deg_s=floats[12],
        angular_velocity_y_deg_s=floats[13],
        angular_velocity_z_deg_s=floats[14],
        steer_deg=floats[15],
        link_id=link_id,
        packet_length=len(packet),
    )


def build_ego_ctrl_cmd(
    *,
    cmd_type: int,
    velocity_kmh: float = 0.0,
    acceleration_mps2: float = 0.0,
    accel: float = 0.0,
    brake: float = 0.0,
    steer_normalized: float = 0.0,
    ctrl_mode: int = 2,
    gear: int = 4,
) -> bytes:
    """공식 EgoCtrlCmd 55바이트 패킷을 생성한다.

    ``steer_normalized``는 실제 앞바퀴각/최대앞바퀴각이며 -1~1이다.
    ``velocity_kmh``는 MORAI UDP 프로토콜 단위인 km/h이다.
    """

    values = (
        b"#MoraiCtrlCmd$",
        23,
        0,
        0,
        0,
        int(ctrl_mode),
        int(gear),
        int(cmd_type),
        float(velocity_kmh),
        float(acceleration_mps2),
        max(0.0, min(1.0, float(accel))),
        max(0.0, min(1.0, float(brake))),
        max(-1.0, min(1.0, float(steer_normalized))),
        b"\r\n",
    )
    packet = struct.pack(EGO_CTRL_CMD_FORMAT, *values)
    if len(packet) != EGO_CTRL_CMD_PACKET_SIZE:
        raise ProtocolError(f"EgoCtrlCmd 생성 길이 오류: {len(packet)}")
    return packet
