# RoboBoat Status Light

## Repository inspection

The initial repository contains `Boat_light.py`, a standalone loop with four
placeholder boolean checks and a six-layer JSON output, and `Boat_lightv2.py`,
which contains only minimal items. There are no package manifests, ROS launch files,
configuration files, or existing tests.

## Confirmed environment and open questions

The inspected machine is Windows with Python 3.14.3. No `ros2` or `colcon`
command, ROS-related environment variables, ROS process, common ROS install
directory, serial port, or connected LED/Arduino/USB-serial device was found.
The selected Python environment has no `rclpy`, `sensor_msgs`, or common
serial/NeoPixel libraries. These checks describe this machine at inspection
time; they do not establish what is installed on the boat or another computer.

No ROS node, topic, message type, thruster command encoding, hardware feedback
source, or LED controller/wiring is confirmed. In particular,
`sensor_msgs/msg/PointCloud2` and the 1100–1900 PWM range are configurable
examples from the project specification, not verified hardware settings.

## Prototype

`status_monitor.py` is a ROS-independent, read-only validation core. Callers
provide discovered node/topic/message facts and component-specific
observations. Unconfigured or unavailable facts remain `GRAY`; configured
limits are not inferred from topic names or hardware assumptions. Thruster
command correctness and physical response are separate facets, and missing
physical feedback stays `GRAY`.

The same module includes an in-memory 16×16 display simulation. Its logical
grid is indexed `grid[y][x]`, starts at the top-left, accepts configurable
component-to-pixel groups, and maps coordinates to a row-major physical index
with optional serpentine rows. It does not access or publish to hardware.

This is not yet a ROS discovery/subscription adapter. A ROS 2 Jazzy machine,
actual topic/node inventory and message definitions, the thruster command
encoding and verified limits, any hardware feedback topic, and the LED
controller/model/wiring are needed before those integrations can be safely
implemented.

## Run the prototype tests

From this directory, run:

```powershell
python -m unittest -v
```

The tests use simulated observations only. To use the validators, construct
`LidarConfig` or `ThrusterConfig` with limits confirmed for the target vehicle,
then pass in observations obtained by a future read-only ROS adapter. Do not
enable physical LED output until its controller interface and wiring have
been identified.
