# Drone SLAM Interface 

A ROS 2 Humble simulation stack for 2D LiDAR SLAM on a quadrotor drone, with a clean Python interface for navigation algorithm development. Built on top of [sjtu_drone](https://github.com/NovoG93/sjtu_drone) and [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox).

---

## What This Is

This repo gives you:
- A Gazebo simulation of a quadrotor drone with a 360° 2D LiDAR sensor
- Real-time 2D SLAM via `slam_toolbox` — the drone builds a map as it flies
- A `drone_slam_interface` package that exposes map and pose data as a simple Python dataclass
- Snapshot scripts for saving the occupancy grid to a file
- A live terminal visualizer

The interface is designed so the navigation team codes against a stable Python API and never touches ROS directly.

---

## System Requirements

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo 11

---

## Setup

### 1. Clone the repo

```bash
git clone <your-repo-url> ~/ros2_ws
cd ~/ros2_ws
```

### 2. Install dependencies

```bash
source /opt/ros/humble/setup.bash

# slam_toolbox
sudo apt install ros-humble-slam-toolbox

# Gazebo ROS 2 bridge
sudo apt install ros-humble-gazebo-ros-pkgs

# xterm (used by sjtu_drone teleop)
sudo apt install xterm

# Gazebo models (needed for the playground world)
cd ~/.gazebo
git clone https://github.com/osrf/gazebo_models models
```

### 3. Install ROS dependencies

```bash
cd ~/ros2_ws
rosdep install -r -y --from-paths src --ignore-src --rosdistro humble
```

### 4. Build

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

Add the source line to your `.bashrc` so you don't have to run it every terminal:

```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

---

## Running the Simulation

Source your workspace:

```bash
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
```

Launch Gazebo + SLAM:

```bash
ros2 launch sjtu_drone_bringup sjtu_drone_bringup.launch.py
```
Wait until Gazebo fully loads before proceeding.

Fly the drone around the environment. The more it explores, the more the map fills in. SLAM will not produce a map until the drone is airborne and the LiDAR sees walls.

---

## Snapshot Scripts

These grab a single snapshot of the current map state and save it to a text file. Run either one while the simulation is running:

```bash
# Visual representation (walls, free space, drone position)
python3 src/drone_slam_interface/drone_slam_interface/save_snapshot_visual.py

# Raw numeric grid (-1, 0, 100 values)
python3 src/drone_slam_interface/drone_slam_interface/save_snapshot_numeric.py
```

Output files are saved to your current working directory as `snapshot_visual.txt` and `snapshot_numeric.txt`.

---

## For the Navigation Team

### Where to put your code

Create your navigation package inside `src/`:

```
ros2_ws/
└── src/
    ├── sjtu_drone/               ← simulation (don't edit)
    ├── drone_slam_interface/     ← SLAM interface (don't edit)
    └── your_nav_package/         ← your code goes here
```

### How to use the interface

Install the interface package into your Python environment by building the workspace (already done if you followed setup). Then in your nav code:

```python
from drone_slam_interface.slam_bridge import SLAMBridge
from drone_slam_interface.slam_types import SLAMState, DronePosition
```

### Startup pattern

Wait once before your loop, then call `get_state()` freely:

```python
from drone_slam_interface.slam_bridge import SLAMBridge

bridge = SLAMBridge()
bridge.start()

# Wait once at startup — blocks until data is confirmed available
bridge.wait_for_pose(timeout=10.0)
bridge.wait_for_map(timeout=10.0)

# Your nav loop — get_state() always returns instantly, never blocks
while True:
    state = bridge.get_state()
    # use state here
```

**Never call `wait_for_pose()` or `wait_for_map()` inside your nav loop.** These are blocking calls for startup only. `get_state()` inside the loop SHOULD always be instant (working on bugfix).

### The SLAMState object

```python
state = bridge.get_state()

state.is_map_ready        # bool — False until drone has scanned enough
state.map_width           # int — number of cells horizontally
state.map_height          # int — number of cells vertically
state.map_resolution      # float — metres per cell (0.05 = 5cm/cell)
state.map_origin_x        # float — world X of bottom-left corner of map
state.map_origin_y        # float — world Y of bottom-left corner of map
state.grid                # List[int] — flat list of all cell values
state.map_timestamp       # float — unix time of last map update
state.position            # DronePosition or None
```

### Cell values

| Value | Meaning |
|---|---|
| `-1` | Unknown — not yet scanned |
| `0` | Free — open space |
| `100` | Occupied — wall or obstacle |

### The DronePosition object

```python
state.position.x          # float — world X in metres
state.position.y          # float — world Y in metres
state.position.yaw        # float — heading in radians (0 = facing +X axis)
state.position.timestamp  # float — unix time of this reading
```

### Helper methods

```python
# Get occupancy value at a grid cell
value = state.get_cell(row, col)        # returns -1 if out of bounds

# Convert world coordinates to grid indices
row, col = state.world_to_grid(x, y)

# Get drone's current grid position
row, col = state.drone_grid_position()  # returns None if no pose yet
```

### Checking data freshness

If your algorithm needs to know how stale the data is:

```python
import time

state = bridge.get_state()

map_age  = time.time() - state.map_timestamp
pose_age = time.time() - state.position.timestamp

if map_age > 2.0:
    print("Map is stale")
if pose_age > 0.5:
    print("Pose is stale")
```

### Minimal working example

```python
from drone_slam_interface.slam_bridge import SLAMBridge
import time

bridge = SLAMBridge()
bridge.start()
bridge.wait_for_pose(timeout=10.0)
bridge.wait_for_map(timeout=10.0)

while True:
    state = bridge.get_state()

    if not state.is_map_ready:
        time.sleep(0.1)
        continue

    row, col = state.drone_grid_position()

    # Check if the cell 10 rows ahead is free
    ahead = state.get_cell(row - 10, col)
    if ahead == 0:
        print("Path ahead is clear")
    elif ahead == 100:
        print("Obstacle ahead")
    else:
        print("Unknown ahead — proceed with caution")

    time.sleep(0.1)
```

---

## Known Limitations

**Drone tilt during movement** — The 2D LiDAR assumes a level horizontal scan. When the drone tilts to move, the LiDAR tilts with it and may hit the floor or ceiling instead of walls, degrading map quality. Fly slowly and keep movements gentle during mapping runs.

**Static map** — Once a cell is marked occupied it stays that way. The map represents everything the drone has ever seen, not necessarily the current state of the environment. Moving obstacles will not be cleared from the map.

**Altitude changes** — The LiDAR scans a single horizontal plane at the drone's current altitude. Significant altitude changes will cause the LiDAR to scan a different slice of the environment, which may produce an inconsistent map. Fly at a consistent altitude during a mapping run.

**Startup delay** — `wait_for_pose()` and `wait_for_map()` block until data is available. If the drone has not taken off yet, these will time out. Always take off before running nav code.

---

## Package Structure

```
ros2_ws/
└── src/
    ├── sjtu_drone/
    │   ├── sjtu_drone_description/
    │   │   └── urdf/
    │   │       └── sjtu_drone.urdf.xacro   ← drone model with 2D LiDAR added
    │   ├── sjtu_drone_bringup/
    │   │   ├── launch/
    │   │   │   └── sjtu_drone_bringup.launch.py  ← main launch file
    │   │   └── config/
    │   │       └── slam_params.yaml         ← slam_toolbox configuration
    │   └── sjtu_drone_control/
    │
    └── drone_slam_interface/
        └── drone_slam_interface/
            ├── slam_types.py               ← SLAMState + DronePosition dataclasses
            ├── slam_bridge.py              ← ROS 2 node, do not import directly
            ├── demo_visualizer.py          ← live terminal map display
            ├── save_snapshot_visual.py     ← saves visual map to txt
            └── save_snapshot_numeric.py    ← saves numeric grid to txt
```

---

## Interface Stability Contract

`slam_types.py` is the stable public interface. The fields and methods on `SLAMState` and `DronePosition` will not change between versions. `slam_bridge.py` is internal — do not import from it directly in nav code except for the `SLAMBridge` class itself.

When this system is ported to a physical drone, only `slam_bridge.py` will change. All nav code written against `SLAMState` will work without modification.
