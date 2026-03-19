"""
save_snapshot_numeric.py

Grabs the current occupancy grid and saves it as a txt file
with raw numbers (-1, 0, 100) and the drone's world coordinates.
No compression — every cell is represented.

Run with:
    python3 save_snapshot_numeric.py
"""

import sys
import os
import time
import math

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from drone_slam_interface.slam_bridge import SLAMBridge

OUTPUT_FILE = 'snapshot_numeric.txt'


def main():
    print("Connecting to SLAMBridge...")
    bridge = SLAMBridge()
    bridge.start()

    print("Waiting for pose...")
    bridge.wait_for_pose(timeout=10.0)

    print("Waiting for map...")
    bridge.wait_for_map(timeout=10.0)

    state = bridge.get_state()

    if not state.is_map_ready:
        print("ERROR: No map data received. Is Gazebo running and has the drone taken off?")
        bridge.stop()
        return

    print(f"Saving {state.map_width}x{state.map_height} grid to {OUTPUT_FILE}...")

    with open(OUTPUT_FILE, 'w') as f:
        f.write("SLAM OCCUPANCY GRID SNAPSHOT\n")
        f.write("=" * 40 + "\n")
        f.write(f"Map size:       {state.map_width} x {state.map_height} cells\n")
        f.write(f"Resolution:     {state.map_resolution} m/cell ({state.map_resolution*100:.0f} cm/cell)\n")
        f.write(f"Map origin:     ({state.map_origin_x:.3f}, {state.map_origin_y:.3f}) metres\n")
        f.write(f"Timestamp:      {state.map_timestamp:.2f}\n")
        f.write("\n")

        f.write("DRONE POSITION\n")
        f.write("-" * 40 + "\n")
        if state.position:
            f.write(f"World X:        {state.position.x:.4f} m\n")
            f.write(f"World Y:        {state.position.y:.4f} m\n")
            f.write(f"Yaw:            {math.degrees(state.position.yaw):.2f} degrees\n")
            f.write(f"Pose timestamp: {state.position.timestamp:.2f}\n")
        else:
            f.write("No position data available.\n")

        f.write("\n")
        f.write("CELL VALUES: -1=unknown  0=free  100=occupied\n")
        f.write("=" * 40 + "\n\n")

        for row in range(state.map_height - 1, -1, -1):
            row_values = []
            for col in range(state.map_width):
                row_values.append(str(state.get_cell(row, col)).rjust(4))
            f.write(''.join(row_values) + "\n")

    print(f"Done. Saved to {OUTPUT_FILE}")
    bridge.stop()


if __name__ == '__main__':
    main()