"""
slam_bridge.py

ROS 2 node that subscribes to /map and /pose from slam_toolbox
and exposes a clean SLAMState dataclass for the navigation team.

Usage:
    from drone_slam_interface.slam_bridge import SLAMBridge
    bridge = SLAMBridge()
    bridge.start()          # non-blocking, spins in background thread
    state = bridge.get_state()
"""

import threading
import time
import math
import copy

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose

from drone_slam_interface.slam_types import SLAMState, DronePosition


class _SLAMNode(Node):
    def __init__(self, on_update=None):
        super().__init__('slam_bridge')
        self._on_update = on_update
        self._lock = threading.Lock()

        # Latest raw pose stored here directly — simple overwrite every message
        self._latest_pose: DronePosition = None

        self._state = SLAMState(
            map_width=0, map_height=0,
            map_resolution=0.05,
            map_origin_x=0.0, map_origin_y=0.0
        )

        # Separate callback groups — map and pose never block each other
        map_group  = MutuallyExclusiveCallbackGroup()
        pose_group = MutuallyExclusiveCallbackGroup()

        # Match /map QoS exactly (TRANSIENT_LOCAL so we get the latched map)
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # Use depth=1 for pose — we only ever care about the latest value
        # KEEP_LAST + depth=1 means old messages are discarded immediately
        pose_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE
        )

        self.create_subscription(OccupancyGrid, '/map', self._map_cb,
            map_qos, callback_group=map_group)
        self.create_subscription(Pose, '/simple_drone/gt_pose', self._pose_cb,
            pose_qos, callback_group=pose_group)

        self.get_logger().info('SLAMBridge node started — waiting for /map and /pose...')

    def _map_cb(self, msg: OccupancyGrid):
        with self._lock:
            self._state.map_width      = msg.info.width
            self._state.map_height     = msg.info.height
            self._state.map_resolution = msg.info.resolution
            self._state.map_origin_x   = msg.info.origin.position.x
            self._state.map_origin_y   = msg.info.origin.position.y
            self._state.grid           = list(msg.data)
            self._state.map_timestamp  = time.time()
            self._state.is_map_ready   = True
            # Snapshot latest pose into state at map update time
            self._state.position = self._latest_pose
        if self._on_update:
            self._on_update(self.get_state())

    def _pose_cb(self, msg: Pose):
        # Just overwrite latest pose — no throttle, no processing delay
        q = msg.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        # Use a separate lock-free store for pose since it updates at 950hz
        # and we don't want map callbacks waiting on pose writes
        self._latest_pose = DronePosition(
            x=msg.position.x,
            y=msg.position.y,
            yaw=yaw,
            timestamp=time.time()
        )

    def get_state(self) -> SLAMState:
        """Returns a snapshot copy of the current SLAMState (thread-safe)."""
        with self._lock:
            # Always inject the freshest pose at read time
            state = copy.deepcopy(self._state)
            state.position = self._latest_pose
            return state


class SLAMBridge:
    """
    High-level interface for the navigation team.
    Wraps the ROS 2 node and exposes a simple get_state() call.
    """

    def __init__(self, on_update=None):
        self._on_update = on_update
        self._node      = None
        self._thread    = None
        self._executor  = None

    def start(self):
        rclpy.init()
        self._node     = _SLAMNode(on_update=self._on_update)
        self._executor = MultiThreadedExecutor(num_threads=4)
        self._executor.add_node(self._node)
        self._thread   = threading.Thread(target=self._executor.spin, daemon=True)
        self._thread.start()
        print("[SLAMBridge] Started.")

    def wait_for_pose(self, timeout=10.0) -> bool:
        """
        Block until first pose is received or timeout is reached.
        Returns True if pose arrived, False if timed out.
        Call this after start() if you need a guaranteed pose before proceeding.
        """
        start = time.time()
        while self._node._latest_pose is None:
            if time.time() - start > timeout:
                print("[SLAMBridge] Warning: timed out waiting for pose.")
                return False
            time.sleep(0.05)
        print("[SLAMBridge] Pose acquired.")
        return True

    def wait_for_map(self, timeout=10.0) -> bool:
        """
        Block until first map is received or timeout is reached.
        Returns True if map arrived, False if timed out.
        """
        start = time.time()
        while not self._node._state.is_map_ready:
            if time.time() - start > timeout:
                print("[SLAMBridge] Warning: timed out waiting for map.")
                return False
            time.sleep(0.05)
        print("[SLAMBridge] Map acquired.")
        return True

    def get_state(self) -> SLAMState:
        """Get the latest SLAMState snapshot."""
        if self._node is None:
            raise RuntimeError("Call start() before get_state()")
        return self._node.get_state()

    def stop(self):
        self._executor.shutdown()
        rclpy.shutdown()