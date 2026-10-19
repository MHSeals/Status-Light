"""ROS-independent status validation and 16x16 LED matrix simulation."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Mapping, Sequence


class HealthState(str, Enum):
    GREEN = "GREEN"
    YELLOW = "YELLOW"
    RED = "RED"
    GRAY = "GRAY"


RGB = tuple[int, int, int]
Pixel = tuple[int, int]

COLORS: Mapping[HealthState, RGB] = {
    HealthState.GREEN: (0, 255, 0),
    HealthState.YELLOW: (255, 180, 0),
    HealthState.RED: (255, 0, 0),
    HealthState.GRAY: (96, 96, 96),
}


@dataclass(frozen=True)
class ComponentStatus:
    name: str
    state: HealthState
    facets: Mapping[str, HealthState]


@dataclass(frozen=True)
class LidarConfig:
    expected_message_type: str | None = None
    expected_frame_id: str | None = None
    min_point_count: int | None = None
    stale_after_sec: float | None = None
    warning_fraction: float = 0.8


@dataclass(frozen=True)
class LidarObservation:
    node_available: bool | None = None
    topic_available: bool | None = None
    message_available: bool | None = None
    message_type: str | None = None
    message_age_sec: float | None = None
    frame_id: str | None = None
    point_count: int | None = None
    coordinates_valid: bool | None = None


@dataclass(frozen=True)
class ThrusterConfig:
    min_pwm: int | None = None
    max_pwm: int | None = None
    neutral_pwm: int | None = None
    expected_pwm: int | None = None
    tolerance: int = 0
    stale_after_sec: float | None = None
    warning_fraction: float = 0.8


@dataclass(frozen=True)
class ThrusterObservation:
    node_available: bool | None = None
    topic_available: bool | None = None
    message_available: bool | None = None
    command_pwm: float | None = None
    message_age_sec: float | None = None
    hardware_response: bool | None = None


def _presence(value: bool | None) -> HealthState:
    if value is None:
        return HealthState.GRAY
    return HealthState.GREEN if value else HealthState.RED


def _configured_checks(checks: Sequence[bool | None]) -> HealthState:
    if any(result is False for result in checks):
        return HealthState.RED
    if not checks or any(result is None for result in checks):
        return HealthState.GRAY
    return HealthState.GREEN


def _freshness(
    age: float | None, timeout: float | None, warning_fraction: float
) -> HealthState:
    if not 0 < warning_fraction < 1:
        raise ValueError("warning_fraction must be greater than 0 and less than 1")
    if age is None or timeout is None:
        return HealthState.GRAY
    if age < 0 or timeout <= 0:
        return HealthState.RED
    if age > timeout:
        return HealthState.RED
    if age >= timeout * warning_fraction:
        return HealthState.YELLOW
    return HealthState.GREEN


def _overall(facets: Mapping[str, HealthState]) -> HealthState:
    states = tuple(facets.values())
    if HealthState.RED in states:
        return HealthState.RED
    if HealthState.YELLOW in states:
        return HealthState.YELLOW
    if not states or HealthState.GRAY in states:
        return HealthState.GRAY
    return HealthState.GREEN


def validate_lidar(
    observation: LidarObservation,
    config: LidarConfig = LidarConfig(),
) -> ComponentStatus:
    """Validate only ROS and message facts that are observed or configured."""
    checks: list[bool | None] = []
    if config.expected_message_type is not None:
        checks.append(
            None
            if observation.message_type is None
            else observation.message_type == config.expected_message_type
        )
    if config.expected_frame_id is not None:
        checks.append(
            None
            if observation.frame_id is None
            else observation.frame_id == config.expected_frame_id
        )
    if config.min_point_count is not None:
        checks.append(
            None
            if observation.point_count is None
            else observation.point_count >= config.min_point_count
        )
    if observation.coordinates_valid is not None:
        checks.append(observation.coordinates_valid)

    facets = {
        "node_availability": _presence(observation.node_available),
        "topic_availability": _presence(observation.topic_available),
        "message_availability": _presence(observation.message_available),
        "data_validity": _configured_checks(checks),
        "data_freshness": _freshness(
            observation.message_age_sec,
            config.stale_after_sec,
            config.warning_fraction,
        ),
    }
    return ComponentStatus("lidar", _overall(facets), facets)


def classify_thruster_command(
    command_pwm: float,
    *,
    min_pwm: int,
    max_pwm: int,
    neutral_pwm: int,
) -> str | None:
    """Return REVERSE, NEUTRAL, or FORWARD; return None outside the configured range."""
    if not min_pwm <= command_pwm <= max_pwm:
        return None
    if command_pwm < neutral_pwm:
        return "REVERSE"
    if command_pwm > neutral_pwm:
        return "FORWARD"
    return "NEUTRAL"


def validate_thruster(
    observation: ThrusterObservation,
    config: ThrusterConfig = ThrusterConfig(),
) -> ComponentStatus:
    checks: list[bool | None] = []
    command = observation.command_pwm
    if config.min_pwm is not None:
        checks.append(None if command is None else command >= config.min_pwm)
    if config.max_pwm is not None:
        checks.append(None if command is None else command <= config.max_pwm)
    if config.expected_pwm is not None:
        checks.append(
            None
            if command is None
            else abs(command - config.expected_pwm) <= config.tolerance
        )

    facets = {
        "node_availability": _presence(observation.node_available),
        "topic_availability": _presence(observation.topic_available),
        "message_availability": _presence(observation.message_available),
        "command_correctness": _configured_checks(checks),
        "command_freshness": _freshness(
            observation.message_age_sec,
            config.stale_after_sec,
            config.warning_fraction,
        ),
        "physical_response": _presence(observation.hardware_response),
    }
    return ComponentStatus("thruster", _overall(facets), facets)


def render_grid(
    statuses: Mapping[str, ComponentStatus],
    pixel_groups: Mapping[str, Sequence[Pixel]],
    *,
    width: int = 16,
    height: int = 16,
) -> list[list[RGB]]:
    """Render an in-memory grid indexed as grid[y][x]; unassigned pixels are gray."""
    if width <= 0 or height <= 0:
        raise ValueError("Grid dimensions must be positive")
    grid = [[COLORS[HealthState.GRAY] for _ in range(width)] for _ in range(height)]
    for name, pixels in pixel_groups.items():
        status = statuses[name]
        color = COLORS[status.state]
        for x, y in pixels:
            if not 0 <= x < width or not 0 <= y < height:
                raise ValueError(f"Pixel ({x}, {y}) is outside the {width}x{height} grid")
            grid[y][x] = color
    return grid


def physical_pixel_index(
    x: int,
    y: int,
    *,
    width: int = 16,
    height: int = 16,
    serpentine: bool = True,
) -> int:
    """Map top-left-origin (x, y) coordinates to a row-major physical LED index."""
    if width <= 0 or height <= 0 or not 0 <= x < width or not 0 <= y < height:
        raise ValueError(f"Pixel ({x}, {y}) is outside the {width}x{height} grid")
    row_x = width - 1 - x if serpentine and y % 2 else x
    return y * width + row_x
