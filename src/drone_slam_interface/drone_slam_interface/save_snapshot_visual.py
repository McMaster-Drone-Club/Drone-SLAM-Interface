"""
save_snapshot_visual.py

Grabs the current occupancy grid and saves it as a txt file
with a visual character representation at full resolution.
Also shows drone position within the map.

Legend:
  #  = occupied (wall/obstacle)
  .  = free (open space)
  ?  = unknown (not yet scanned)
  D  = drone position

Run with:
    python3 save_snapshot_visual.py
"""

import sys
import os
import time
import math

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from drone_slam_interface.slam_bridge import SLAMBridge

OUTPUT_FILE = 'snapshot_visual.txt'

CHAR_OCCUPIED = '#'
CHAR_FREE     = '.'
CHAR_UNKNOWN  = '?'
CHAR_DRONE    = 'D'


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

    drone_cell = state.drone_grid_position()

    with open(OUTPUT_FILE, 'w') as f:
        f.write("SLAM OCCUPANCY GRID SNAPSHOT — VISUAL\n")
        f.write("=" * 40 + "\n")
        f.write(f"Map size:    {state.map_width} x {state.map_height} cells\n")
        f.write(f"Resolution:  {state.map_resolution} m/cell ({state.map_resolution*100:.0f} cm/cell)\n")
        f.write(f"Map origin:  ({state.map_origin_x:.3f}, {state.map_origin_y:.3f}) metres\n")
        f.write(f"Timestamp:   {state.map_timestamp:.2f}\n")
        f.write("\n")

        f.write("DRONE POSITION\n")
        f.write("-" * 40 + "\n")
        if state.position:
            f.write(f"World:       ({state.position.x:.4f} m,  {state.position.y:.4f} m)\n")
            f.write(f"Yaw:         {math.degrees(state.position.yaw):.2f} degrees\n")
            if drone_cell:
                f.write(f"Grid cell:   row={drone_cell[0]}, col={drone_cell[1]}\n")
        else:
            f.write("No position data available.\n")

        f.write("\n")
        f.write(f"Legend:  {CHAR_DRONE}=drone  {CHAR_OCCUPIED}=wall  {CHAR_FREE}=free  {CHAR_UNKNOWN}=unknown\n")
        f.write("=" * 40 + "\n\n")

        for row in range(state.map_height - 1, -1, -1):
            row_chars = []
            for col in range(state.map_width):
                if drone_cell and row == drone_cell[0] and col == drone_cell[1]:
                    row_chars.append(CHAR_DRONE)
                else:
                    val = state.get_cell(row, col)
                    if val == 100:
                        row_chars.append(CHAR_OCCUPIED)
                    elif val == 0:
                        row_chars.append(CHAR_FREE)
                    else:
                        row_chars.append(CHAR_UNKNOWN)
            f.write(''.join(row_chars) + "\n")

    print(f"Done. Saved to {OUTPUT_FILE}")
    bridge.stop()


if __name__ == '__main__':
    main()