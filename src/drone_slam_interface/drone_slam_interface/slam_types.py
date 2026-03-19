from dataclasses import dataclass, field
from typing import List, Optional
import time


@dataclass
class DronePosition:
    """Drone's estimated position within the map."""
    x: float          # metres
    y: float          # metres
    yaw: float        # radians, 0 = facing +x axis
    timestamp: float  # unix time


@dataclass
class SLAMState:
    """
    The stable data contract between the SLAM middleware and the navigation team.

    Navigation team: code against this object. Do not import anything from rclpy.
    The internals (ROS version, SLAM library) may change - this interface will not.
    """
    # Map data
    map_width: int              # number of cells horizontally
    map_height: int             # number of cells vertically
    map_resolution: float       # metres per cell (e.g. 0.05 = 5cm per cell)
    map_origin_x: float         # world X coordinate of cell (0,0)
    map_origin_y: float         # world Y coordinate of cell (0,0)

    # Occupancy grid: list of (map_width * map_height) integers
    # -1  = unknown
    #  0  = free
    # 100 = occupied
    grid: List[int] = field(default_factory=list)

    # Drone pose
    position: Optional[DronePosition] = None

    # Meta
    map_timestamp: float = field(default_factory=time.time)
    is_map_ready: bool = False

    def get_cell(self, row: int, col: int) -> int:
        """
        Get occupancy value at grid row/col.
        Returns -1 (unknown) if out of bounds.
        """
        if row < 0 or row >= self.map_height or col < 0 or col >= self.map_width:
            return -1
        return self.grid[row * self.map_width + col]

    def world_to_grid(self, world_x: float, world_y: float):
        """Convert world coordinates (metres) to (row, col) grid indices."""
        col = int((world_x - self.map_origin_x) / self.map_resolution)
        row = int((world_y - self.map_origin_y) / self.map_resolution)
        return row, col

    def drone_grid_position(self):
        """Returns (row, col) of the drone in the grid, or None if no pose yet."""
        if self.position is None:
            return None
        return self.world_to_grid(self.position.x, self.position.y)