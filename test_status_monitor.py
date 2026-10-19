import unittest

from status_monitor import (
    COLORS,
    ComponentStatus,
    HealthState,
    LidarConfig,
    LidarObservation,
    ThrusterConfig,
    ThrusterObservation,
    classify_thruster_command,
    physical_pixel_index,
    render_grid,
    validate_lidar,
    validate_thruster,
)


class StatusMonitorTests(unittest.TestCase):
    def test_lidar_healthy_and_faulty_inputs(self) -> None:
        config = LidarConfig(
            expected_message_type="sensor_msgs/msg/PointCloud2",
            expected_frame_id="lidar",
            min_point_count=1,
            stale_after_sec=1.0,
        )
        healthy = validate_lidar(
            LidarObservation(
                node_available=True,
                topic_available=True,
                message_available=True,
                message_type="sensor_msgs/msg/PointCloud2",
                message_age_sec=0.1,
                frame_id="lidar",
                point_count=100,
                coordinates_valid=True,
            ),
            config,
        )
        faulty = validate_lidar(
            LidarObservation(
                node_available=True,
                topic_available=True,
                message_available=True,
                message_type="sensor_msgs/msg/PointCloud2",
                message_age_sec=2.0,
                frame_id="lidar",
                point_count=0,
                coordinates_valid=False,
            ),
            config,
        )

        self.assertEqual(healthy.state, HealthState.GREEN)
        self.assertEqual(faulty.state, HealthState.RED)
        self.assertEqual(faulty.facets["data_validity"], HealthState.RED)
        self.assertEqual(faulty.facets["data_freshness"], HealthState.RED)

    def test_unconfigured_lidar_assumptions_remain_unknown(self) -> None:
        status = validate_lidar(
            LidarObservation(node_available=True, topic_available=True)
        )
        self.assertEqual(status.state, HealthState.GRAY)
        self.assertEqual(status.facets["data_validity"], HealthState.GRAY)

    def test_confirmed_failure_is_not_hidden_by_unknown_data(self) -> None:
        status = validate_lidar(
            LidarObservation(coordinates_valid=False),
            LidarConfig(min_point_count=1),
        )
        self.assertEqual(status.facets["data_validity"], HealthState.RED)

    def test_freshness_warning_threshold_is_configurable(self) -> None:
        status = validate_lidar(
            LidarObservation(message_age_sec=0.7),
            LidarConfig(stale_after_sec=1.0, warning_fraction=0.6),
        )
        self.assertEqual(status.facets["data_freshness"], HealthState.YELLOW)

    def test_thruster_command_checks_do_not_claim_hardware_response(self) -> None:
        config = ThrusterConfig(
            min_pwm=1100,
            max_pwm=1900,
            neutral_pwm=1500,
            expected_pwm=1600,
            tolerance=5,
            stale_after_sec=1.0,
        )
        command_only = validate_thruster(
            ThrusterObservation(
                node_available=True,
                topic_available=True,
                message_available=True,
                command_pwm=1602,
                message_age_sec=0.1,
            ),
            config,
        )
        faulty = validate_thruster(
            ThrusterObservation(
                node_available=True,
                topic_available=True,
                message_available=True,
                command_pwm=2000,
                message_age_sec=0.1,
                hardware_response=False,
            ),
            config,
        )

        self.assertEqual(command_only.facets["command_correctness"], HealthState.GREEN)
        self.assertEqual(command_only.facets["physical_response"], HealthState.GRAY)
        self.assertEqual(command_only.state, HealthState.GRAY)
        self.assertEqual(faulty.facets["command_correctness"], HealthState.RED)
        self.assertEqual(faulty.facets["physical_response"], HealthState.RED)
        self.assertEqual(faulty.state, HealthState.RED)

    def test_thruster_direction_and_pwm_limits(self) -> None:
        limits = {"min_pwm": 1100, "max_pwm": 1900, "neutral_pwm": 1500}
        self.assertEqual(classify_thruster_command(1100, **limits), "REVERSE")
        self.assertEqual(classify_thruster_command(1499, **limits), "REVERSE")
        self.assertEqual(classify_thruster_command(1500, **limits), "NEUTRAL")
        self.assertEqual(classify_thruster_command(1501, **limits), "FORWARD")
        self.assertEqual(classify_thruster_command(1900, **limits), "FORWARD")
        self.assertIsNone(classify_thruster_command(1901, **limits))

    def test_grid_and_serpentine_pixel_mapping(self) -> None:
        status = ComponentStatus("lidar", HealthState.GREEN, {})
        grid = render_grid(
            {"lidar": status},
            {"lidar": [(0, y) for y in range(16)]},
        )
        self.assertEqual(len(grid), 16)
        self.assertTrue(all(len(row) == 16 for row in grid))
        self.assertTrue(all(grid[y][0] == COLORS[HealthState.GREEN] for y in range(16)))
        self.assertEqual(physical_pixel_index(0, 1), 31)
        self.assertEqual(physical_pixel_index(0, 1, serpentine=False), 16)


if __name__ == "__main__":
    unittest.main()
