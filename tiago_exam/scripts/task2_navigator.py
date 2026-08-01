#!/usr/bin/env python3
"""
Task 2: ArUco Navigator.
Localizes from a random start pose, autonomously discovers the pick
(ID 26) and place (ID 238) ArUco markers, and navigates to a safe
standoff position in front of each.
"""

import math
import random
import subprocess
import time
from threading import Lock

import numpy as np
import PyKDL as kdl

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from aruco_msgs.msg import MarkerArray
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from tf2_ros import Buffer, TransformListener


def kdl_from_msg(translation, rotation) -> kdl.Frame:
    return kdl.Frame(
        kdl.Rotation.Quaternion(rotation.x, rotation.y, rotation.z, rotation.w),
        kdl.Vector(translation.x, translation.y, translation.z))


def kdl_to_pose_stamped(frame: kdl.Frame, parent_frame: str, stamp) -> PoseStamped:
    ps = PoseStamped()
    ps.header.frame_id = parent_frame
    ps.header.stamp = stamp
    ps.pose.position.x = frame.p.x()
    ps.pose.position.y = frame.p.y()
    ps.pose.position.z = frame.p.z()
    qx, qy, qz, qw = frame.M.GetQuaternion()
    ps.pose.orientation.x = qx
    ps.pose.orientation.y = qy
    ps.pose.orientation.z = qz
    ps.pose.orientation.w = qw
    return ps


class ArucoNavigator(Node):

    # Target and detection parameters
    PICK_ID = 26
    PLACE_ID = 238
    APPROACH_DIST = 0.6
    MARKER_MIN_RANGE = 0.3
    MARKER_MAX_RANGE = 6.0
    MARKER_MIN_HEIGHT = 0.0
    MARKER_MAX_HEIGHT = 2.2
    CONSISTENCY_RADIUS = 0.5
    CONSISTENCY_COUNT = 3

    # Search / patrol
    SEARCH_SPEED = 0.25
    DISCOVER_SPIN = (2 * math.pi / SEARCH_SPEED) * 1.15
    PATROL_COUNT = 8
    WALL_CLEARANCE_M = 0.8
    PATROL_MIN_SPACING = 2.0
    PATROL_MAX_ATTEMPTS = 400

    # Localization - strict convergence
    AMCL_COV_XY = 0.07
    AMCL_COV_YAW = 0.15
    LOCALIZE_SPIN_SPEED = 0.4
    LOCALIZE_MIN_ROTATIONS = 2.5
    LOCALIZE_MAX_DURATION = 150.0
    LOCALIZE_RETRIGGER_EVERY = 40.0
    LOCALIZE_NUDGE_SPEED = 0.15
    LOCALIZE_NUDGE_DURATION = 1.5

    # Head pitch
    HEAD_PITCH_NAV = -0.35
    HEAD_PITCH_SEARCH = -0.05

    # Safety
    SAFE_DIST = 0.40
    FRONT_CONE_DEG = 32.0
    EMERGENCY_DEBOUNCE = 3
    BACKUP_DURATION = 1.2
    ESCAPE_REAR_CLEAR_MIN = 0.50
    ESCAPE_ROTATE_SPEED = 0.6
    ESCAPE_FORWARD_SPEED = 0.15
    ESCAPE_FORWARD_DURATION = 1.0
    ESCAPE_SECTOR_COUNT = 12
    ESCAPE_MAX_ROTATE_DURATION = 4.0
    DANGER_ZONE_CLEARANCE = 1.5
    MAX_CONSECUTIVE_EMERGENCIES = 3
    RECOVERY_ROTATE_DURATION = 3.0
    CLOSE_ENOUGH_DIST = 1.20

    def __init__(self):
        super().__init__('aruco_navigator')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.lock = Lock()

        self.approach_poses = {}
        self.marker_frames = {}
        self.detection_history = {}
        self.latest_map = None
        self.map_bounds = None
        self.patrol_targets = []
        self.patrol_index = 0
        self._visited_patrol_points = []

        self.state = 'INIT'
        self.busy = False
        self.head_mode = None
        self.arm_folded = False
        self.localize_triggered = False
        self.amcl_converged = False
        self.spin_started_at = None
        self.current_goal_id = None
        self._current_goal_handle = None
        self.amcl_cov = None

        self.rotation_accumulated = 0.0
        self._last_localize_tick = None
        self._last_retrigger_at = 0.0
        self._cov_at_last_check = None
        self._localize_rotations_done = 0
        self._localize_nudge_until = None
        self._localize_nudge_dir = 1
        self._localize_phase_start = None

        self.emergency = False
        self._obstacle_hits = 0
        self._consecutive_emergencies = 0
        self._nav_active_goal_ids = ('pick', 'place', 'patrol')
        self._last_scan = None
        self._last_amcl_xy = None
        self._danger_zones = []
        self._pending_action = None

        # Scanning state: 0=idle/patrol, 1=scanning_left, 2=scanning_right
        self._scan_phase = 0
        self._scan_start_time = None
        self._marker_found = False

        self.cmd_vel_pub = self.create_publisher(
            Twist, '/mobile_base_controller/cmd_vel_unstamped', 10)

        map_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE)

        self.create_subscription(OccupancyGrid, '/map', self._on_map, map_qos)
        self.create_subscription(MarkerArray, '/aruco_station/markers',
                                 self._on_markers, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',
                                 self._on_amcl, 10)
        self.create_subscription(LaserScan, '/scan_raw', self._on_scan, 10)

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.head_client = ActionClient(self, FollowJointTrajectory,
                                        'head_controller/follow_joint_trajectory')
        self.arm_client = ActionClient(self, FollowJointTrajectory,
                                       'arm_controller/follow_joint_trajectory')
        self.relocalize_cli = self.create_client(
            Empty, '/reinitialize_global_localization')

        self.declare_parameter('head_pitch_nav', self.HEAD_PITCH_NAV)
        self.declare_parameter('head_pitch_search', self.HEAD_PITCH_SEARCH)

        self.declare_parameter('test_teleport', False)
        self.declare_parameter('test_teleport_x', 0.0)
        self.declare_parameter('test_teleport_y', 0.0)
        self.declare_parameter('test_teleport_yaw', 0.0)
        self.declare_parameter('test_teleport_model', 'tiago')
        self._did_test_teleport = False

        self._main_timer = self.create_timer(0.5, self._tick)
        self.get_logger().info(
            f'ArUco navigator ready. Seeking pick={self.PICK_ID}, '
            f'place={self.PLACE_ID}.')

    def _pitch_nav(self) -> float:
        return float(self.get_parameter('head_pitch_nav').value)

    def _pitch_search(self) -> float:
        return float(self.get_parameter('head_pitch_search').value)

    def _scan_head(self, pan, pitch, mode):
        """Move head to a specific pan (horizontal) and pitch (vertical) angle."""
        if not self.head_client.wait_for_server(timeout_sec=1.0):
            return
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = ['head_1_joint', 'head_2_joint']
        pt = JointTrajectoryPoint()
        pt.positions = [pan, pitch]
        pt.time_from_start.sec = 1
        goal.trajectory.points.append(pt)
        fut = self.head_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)

    def _maybe_test_teleport(self):
        if self._did_test_teleport:
            return
        self._did_test_teleport = True
        if not self.get_parameter('test_teleport').value:
            return
        x = float(self.get_parameter('test_teleport_x').value)
        y = float(self.get_parameter('test_teleport_y').value)
        yaw = float(self.get_parameter('test_teleport_yaw').value)
        model = self.get_parameter('test_teleport_model').value
        cmd = ['gz', 'model', '-m', model,
               '-x', str(x), '-y', str(y), '-z', '0.0', '-Y', str(yaw)]
        try:
            subprocess.run(cmd, timeout=5.0, check=True,
                           capture_output=True, text=True)
        except Exception as e:
            self.get_logger().error(f'Teleport failed: {e}')

    def _on_map(self, msg: OccupancyGrid):
        self.latest_map = msg
        res = msg.info.resolution
        ox = msg.info.origin.position.x
        oy = msg.info.origin.position.y
        w, h = msg.info.width, msg.info.height
        data = np.array(msg.data, dtype=np.int8).reshape((h, w))
        free = np.argwhere(data == 0)
        if free.size > 0:
            rmin, cmin = free.min(axis=0)
            rmax, cmax = free.max(axis=0)
            margin = 0.8
            self.map_bounds = (
                ox + cmin * res + margin,
                ox + cmax * res - margin,
                oy + rmin * res + margin,
                oy + rmax * res - margin,
            )

    def _on_amcl(self, msg: PoseWithCovarianceStamped):
        c = msg.pose.covariance
        self.amcl_cov = (c[0], c[7], c[35])
        self._last_amcl_xy = (msg.pose.pose.position.x,
                              msg.pose.pose.position.y)
        if not self.amcl_converged:
            cx, cy, cyaw = self.amcl_cov
            self.get_logger().info(
                f'AMCL cov x={cx:.4f} y={cy:.4f} yaw={cyaw:.4f}',
                throttle_duration_sec=2.0)
            if cx < self.AMCL_COV_XY and cy < self.AMCL_COV_XY \
                    and cyaw < self.AMCL_COV_YAW:
                self.amcl_converged = True
                self.get_logger().info(
                    f'AMCL converged. cov=({cx:.4f}, {cy:.4f}, {cyaw:.4f})')

    def _on_scan(self, msg: LaserScan):
        self._last_scan = msg
        n = len(msg.ranges)
        if n == 0:
            return
        half_cone = math.radians(self.FRONT_CONE_DEG)
        close = False
        for i in range(n):
            r = msg.ranges[i]
            if not (msg.range_min < r < 10.0):
                continue
            angle = msg.angle_min + i * msg.angle_increment
            if abs(angle) <= half_cone and r < self.SAFE_DIST:
                close = True
                break

        self._obstacle_hits = self._obstacle_hits + 1 if close else 0
        if self._obstacle_hits >= self.EMERGENCY_DEBOUNCE:
            self.emergency = True

    def _on_markers(self, msg: MarkerArray):
        if not msg.markers:
            return
        src_frame = msg.header.frame_id
        try:
            tf_map = self.tf_buffer.lookup_transform(
                'map', src_frame, rclpy.time.Time(),
                timeout=Duration(seconds=0.5))
        except Exception:
            return

        f_map_src = kdl_from_msg(tf_map.transform.translation,
                                 tf_map.transform.rotation)

        with self.lock:
            for m in msg.markers:
                if m.id not in (self.PICK_ID, self.PLACE_ID):
                    continue
                if m.id in self.approach_poses:
                    continue

                px = m.pose.pose.position.x
                py = m.pose.pose.position.y
                pz = m.pose.pose.position.z

                planar_range = math.hypot(px, py)
                if not (self.MARKER_MIN_RANGE < planar_range < self.MARKER_MAX_RANGE):
                    continue
                if not (self.MARKER_MIN_HEIGHT < pz < self.MARKER_MAX_HEIGHT):
                    continue

                f_marker = kdl_from_msg(m.pose.pose.position, m.pose.pose.orientation)
                f_in_map = f_map_src * f_marker
                mx, my = f_in_map.p.x(), f_in_map.p.y()

                hist = self.detection_history.setdefault(m.id, [])
                hist.append((mx, my))
                if len(hist) > self.CONSISTENCY_COUNT:
                    hist.pop(0)
                if len(hist) < self.CONSISTENCY_COUNT:
                    continue

                avg_x = sum(p[0] for p in hist) / len(hist)
                avg_y = sum(p[1] for p in hist) / len(hist)
                if any(math.hypot(p[0]-avg_x, p[1]-avg_y) > self.CONSISTENCY_RADIUS
                       for p in hist):
                    hist.clear()
                    continue

                self.marker_frames[m.id] = f_in_map
                self.approach_poses[m.id] = self._compute_approach(
                    f_in_map, viewer_xy=self._last_amcl_xy)
                self._marker_found = True
                self.get_logger().info(
                    f'Locked marker {m.id} at ({mx:.2f},{my:.2f})')

    def _compute_approach(self, marker_frame: kdl.Frame, extra_dist=0.0,
                          viewer_xy=None) -> PoseStamped:
        if viewer_xy is not None:
            dx = viewer_xy[0] - marker_frame.p.x()
            dy = viewer_xy[1] - marker_frame.p.y()
            n = math.hypot(dx, dy)
            ox, oy = (1.0, 0.0) if n < 1e-6 else (dx / n, dy / n)
        else:
            outward = marker_frame.M * kdl.Vector(0.0, 0.0, 1.0)
            ox, oy = outward.x(), outward.y()
            n = math.hypot(ox, oy)
            if n >= 1e-6:
                ox, oy = ox / n, oy / n
            else:
                ox, oy = 1.0, 0.0
        dist = self.APPROACH_DIST + extra_dist
        gx = marker_frame.p.x() + ox * dist
        gy = marker_frame.p.y() + oy * dist
        yaw = math.atan2(-oy, -ox)
        approach = kdl.Frame(kdl.Rotation.RotZ(yaw), kdl.Vector(gx, gy, 0.0))
        return kdl_to_pose_stamped(approach, 'map', self.get_clock().now().to_msg())

    def _check_close_enough(self, target_id, station_name, next_state) -> bool:
        if target_id not in self.marker_frames or self._last_amcl_xy is None:
            return False
        mf = self.marker_frames[target_id]
        dist = math.hypot(self._last_amcl_xy[0] - mf.p.x(),
                          self._last_amcl_xy[1] - mf.p.y())
        if dist > self.CLOSE_ENOUGH_DIST:
            return False

        # Clear state transition banner
        self.get_logger().info('==================================================')
        self.get_logger().info(f'STATION REACHED: {station_name}')
        self.get_logger().info(f'Distance: {dist:.2f}m from marker {target_id}')
        self.get_logger().info(f'State -> {next_state}')
        self.get_logger().info('==================================================')

        if self.busy:
            self._cancel_goal()
            self.busy = False
            self.current_goal_id = None
        self.state = next_state
        self.spin_started_at = self.get_clock().now()
        return True

    def _tick(self):
        if self.emergency:
            self.emergency = False
            self._obstacle_hits = 0
            self._consecutive_emergencies += 1

            if self.busy and self.current_goal_id in self._nav_active_goal_ids:
                self.get_logger().warn('Obstacle detected. Cancelling goal.')
                self._cancel_goal()
            else:
                self.get_logger().warn('Obstacle detected. Recovering.')

            self._stop()

            escalated = self._consecutive_emergencies >= self.MAX_CONSECUTIVE_EMERGENCIES
            if escalated:
                self._consecutive_emergencies = 0
            self._recover_from_obstacle(escalated)

            self.busy = False
            self.current_goal_id = None
            self.spin_started_at = self.get_clock().now()
            return

        if self.busy and self.current_goal_id == 'patrol':
            target_id = (self.PICK_ID if self.state == 'DISCOVER_PICK'
                         else self.PLACE_ID)
            if target_id in self.approach_poses:
                self._cancel_goal()
                return

        if self.state == 'GOTO_PICK':
            if self._check_close_enough(self.PICK_ID, 'Pick station',
                                        'DISCOVER_PLACE'):
                self.patrol_index = 0
                self.patrol_targets = []
                return
        elif self.state == 'GOTO_PLACE':
            if self._check_close_enough(self.PLACE_ID, 'Place station', 'DONE'):
                return

        if self.busy:
            return

        if self._pending_action is not None:
            return

        if self.state == 'INIT':
            self._maybe_test_teleport()
            if not self.nav_client.wait_for_server(timeout_sec=0.5):
                return
            if not self.arm_folded:
                self._fold_arm()
                return
            if self.head_mode != 'nav':
                self._tilt_head(self._pitch_nav(), 'nav')
                return
            self._trigger_localize()
            self.spin_started_at = self.get_clock().now()
            self._localize_phase_start = self.get_clock().now()
            self._last_localize_tick = time.time()
            self.rotation_accumulated = 0.0
            self._cov_at_last_check = None
            self.state = 'LOCALIZE'
            self.get_logger().info('State: LOCALIZE')

        elif self.state == 'LOCALIZE':
            elapsed = self._elapsed()
            localize_elapsed = (
                (self.get_clock().now() - self._localize_phase_start).nanoseconds * 1e-9
                if self._localize_phase_start is not None else elapsed)
            min_rotation_needed = self.LOCALIZE_MIN_ROTATIONS * 2 * math.pi

            if self.amcl_converged:
                self._stop()
                self._finish_localize()
                return

            if localize_elapsed >= self.LOCALIZE_MAX_DURATION:
                self._stop()
                self.get_logger().warn(
                    f'AMCL not converged after {self.LOCALIZE_MAX_DURATION:.0f}s '
                    f'proceeding anyway.')
                self._finish_localize()
                return

            if (self.rotation_accumulated >= min_rotation_needed
                    and elapsed - self._last_retrigger_at >= self.LOCALIZE_RETRIGGER_EVERY):
                cov_metric = sum(self.amcl_cov) if self.amcl_cov is not None else float('inf')
                improved = (self._cov_at_last_check is None
                            or cov_metric < self._cov_at_last_check * 0.7)
                self._cov_at_last_check = cov_metric
                self._last_retrigger_at = elapsed

                if not improved:
                    self.get_logger().warn('Covariance stagnant. Retriggering localization.')
                    self.localize_triggered = False
                    self._trigger_localize()

            self._spin_localize()

        elif self.state == 'DISCOVER_PICK':
            if self.PICK_ID in self.approach_poses:
                self._stop()
                self.state = 'GOTO_PICK'
                self.get_logger().info('==================================================')
                self.get_logger().info('State: DISCOVER_PICK -> GOTO_PICK')
                self.get_logger().info('==================================================')
                self._scan_phase = 0
                self._marker_found = False
            else:
                self._discover_step()

        elif self.state == 'GOTO_PICK':
            if self.head_mode != 'search':
                self._tilt_head(self._pitch_search(), 'search')
                return
            if not self.busy:
                self.current_goal_id = 'pick'
                self._send_goal(self.approach_poses[self.PICK_ID])

        elif self.state == 'DISCOVER_PLACE':
            if self.PLACE_ID in self.approach_poses:
                self._stop()
                self.state = 'GOTO_PLACE'
                self.get_logger().info('==================================================')
                self.get_logger().info('State: DISCOVER_PLACE -> GOTO_PLACE')
                self.get_logger().info('==================================================')
                self._scan_phase = 0
                self._marker_found = False
            else:
                self._discover_step()

        elif self.state == 'GOTO_PLACE':
            if self.head_mode != 'search':
                self._tilt_head(self._pitch_search(), 'search')
                return
            if not self.busy:
                self.current_goal_id = 'place'
                self._send_goal(self.approach_poses[self.PLACE_ID])

        elif self.state == 'DONE':
            self.get_logger().info('==================================================')
            self.get_logger().info('TASK 2 COMPLETE: Both stations found.')
            self.get_logger().info('==================================================')
            self._save_yaml()
            self._main_timer.cancel()

    def _finish_localize(self):
        self.patrol_index = 0
        self.spin_started_at = self.get_clock().now()
        self._build_patrol()
        self.state = 'DISCOVER_PICK'
        self.get_logger().info('==================================================')
        self.get_logger().info(f'State: DISCOVER_PICK (marker {self.PICK_ID})')
        self.get_logger().info('==================================================')

    def _discover_step(self):
        # If marker is already found, stop scanning and go to GOTO
        if self._marker_found:
            return

        if self._scan_phase == 0:
            # Normal patrol: spin, then stop at a waypoint
            if self._elapsed() < self.DISCOVER_SPIN:
                if self.head_mode != 'search':
                    self._tilt_head(self._pitch_search(), 'search')
                    return
                self._spin()
                return
            self._stop()
            if self.head_mode != 'nav':
                self._tilt_head(self._pitch_nav(), 'nav')
                return
            if not self.patrol_targets:
                self._build_patrol()
            if not self.patrol_targets:
                self.get_logger().warn('No patrol targets. Spinning more.')
                self.spin_started_at = self.get_clock().now()
                return

            # At waypoint – start left scan (45°)
            self._scan_phase = 1
            self._scan_start_time = self.get_clock().now()
            self.get_logger().info('Scanning: 45 degrees left')
            self._scan_head(0.4, self._pitch_search(), 'scan_left')
            return

        elif self._scan_phase == 1:
            # Left scan done – wait, then move to right scan (90°)
            elapsed = (self.get_clock().now() - self._scan_start_time).nanoseconds * 1e-9
            if elapsed < 1.5:
                return
            self._scan_phase = 2
            self._scan_start_time = self.get_clock().now()
            self.get_logger().info('Scanning: 90 degrees right')
            self._scan_head(-0.8, self._pitch_search(), 'scan_right')
            return

        elif self._scan_phase == 2:
            # Right scan done – wait, then send patrol goal
            elapsed = (self.get_clock().now() - self._scan_start_time).nanoseconds * 1e-9
            if elapsed < 1.5:
                return
            self._scan_phase = 0
            self.get_logger().info('Scan complete – moving to next waypoint')

            tx, ty = self.patrol_targets[self.patrol_index]
            self.patrol_index = (self.patrol_index + 1) % len(self.patrol_targets)
            if self.patrol_index == 0:
                self._build_patrol()
            goal_frame = kdl.Frame(kdl.Rotation.Identity(), kdl.Vector(tx, ty, 0.0))
            pose = kdl_to_pose_stamped(goal_frame, 'map', self.get_clock().now().to_msg())
            self.current_goal_id = 'patrol'
            self._send_goal(pose)

    def _build_patrol(self):
        if self.latest_map is None:
            self.patrol_targets = []
            return
        grid = self.latest_map
        res = grid.info.resolution
        ox = grid.info.origin.position.x
        oy = grid.info.origin.position.y
        w, h = grid.info.width, grid.info.height
        data = np.array(grid.data, dtype=np.int8).reshape((h, w))

        obstacle = data != 0
        clearance_cells = max(1, int(math.ceil(self.WALL_CLEARANCE_M / res)))
        pad = clearance_cells
        padded = np.pad(obstacle, pad, mode='constant', constant_values=True)
        h_dil = np.zeros_like(padded)
        for dx in range(-pad, pad + 1):
            h_dil |= np.roll(padded, dx, axis=1)
        v_dil = np.zeros_like(h_dil)
        for dy in range(-pad, pad + 1):
            v_dil |= np.roll(h_dil, dy, axis=0)
        dilated_obstacle = v_dil[pad:-pad, pad:-pad]

        free = (data == 0) & (~dilated_obstacle)
        idx = np.argwhere(free)
        if idx.size == 0:
            idx = np.argwhere(data == 0)
        if idx.size == 0:
            self.patrol_targets = []
            return

        pts = []
        for cell in idx:
            j, i = cell
            x = ox + (i + 0.5) * res
            y = oy + (j + 0.5) * res
            if self.map_bounds:
                xmin, xmax, ymin, ymax = self.map_bounds
                if not (xmin < x < xmax and ymin < y < ymax):
                    continue
            pts.append((x, y))
        if not pts:
            pts = [(ox + (c[1] + 0.5) * res, oy + (c[0] + 0.5) * res)
                   for c in np.argwhere(data == 0)]

        arr = np.array(pts)
        rng = random.Random()
        VISITED_CLEARANCE = self.PATROL_MIN_SPACING * 0.75

        def _far_enough(cand, others, min_d):
            return all(math.hypot(cand[0] - c[0], cand[1] - c[1]) >= min_d
                       for c in others)

        visited = list(self._visited_patrol_points)
        danger = list(self._danger_zones)

        chosen = []
        attempts = 0
        while len(chosen) < self.PATROL_COUNT and attempts < self.PATROL_MAX_ATTEMPTS:
            attempts += 1
            cand = arr[rng.randrange(len(arr))]
            if (_far_enough(cand, chosen, self.PATROL_MIN_SPACING)
                    and _far_enough(cand, visited, VISITED_CLEARANCE)
                    and _far_enough(cand, danger, self.DANGER_ZONE_CLEARANCE)):
                chosen.append(cand)

        if len(chosen) < self.PATROL_COUNT:
            if len(visited) > self.PATROL_COUNT:
                visited = visited[len(visited) // 2:]
            attempts = 0
            while len(chosen) < self.PATROL_COUNT and attempts < self.PATROL_MAX_ATTEMPTS:
                attempts += 1
                cand = arr[rng.randrange(len(arr))]
                if (_far_enough(cand, chosen, self.PATROL_MIN_SPACING)
                        and _far_enough(cand, visited, VISITED_CLEARANCE)
                        and _far_enough(cand, danger, self.DANGER_ZONE_CLEARANCE)):
                    chosen.append(cand)

        if len(chosen) < self.PATROL_COUNT:
            while len(chosen) < self.PATROL_COUNT and len(arr) > 0:
                chosen.append(arr[rng.randrange(len(arr))])

        self.patrol_targets = [(float(p[0]), float(p[1])) for p in chosen]
        self._visited_patrol_points.extend(self.patrol_targets)

    def _fold_arm(self):
        if self.arm_folded or self._pending_action is not None:
            return
        if not self.arm_client.wait_for_server(timeout_sec=1.0):
            return
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = [
            'arm_1_joint', 'arm_2_joint', 'arm_3_joint', 'arm_4_joint',
            'arm_5_joint', 'arm_6_joint', 'arm_7_joint']
        pt = JointTrajectoryPoint()
        pt.positions = [2.6, -1.5, 0.6, 2.0, 1.2, -1.39, 2.0]
        pt.time_from_start.sec = 5
        goal.trajectory.points.append(pt)
        self._pending_action = 'arm'
        fut = self.arm_client.send_goal_async(goal)
        fut.add_done_callback(self._on_arm_goal_response)

    def _on_arm_goal_response(self, future):
        gh = future.result()
        if gh is None or not gh.accepted:
            self._pending_action = None
            return
        gh.get_result_async().add_done_callback(self._on_arm_goal_result)

    def _on_arm_goal_result(self, future):
        self.arm_folded = True
        self._pending_action = None

    def _tilt_head(self, pitch, mode):
        if self.head_mode == mode or self._pending_action is not None:
            return
        if not self.head_client.wait_for_server(timeout_sec=1.0):
            return
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = ['head_1_joint', 'head_2_joint']
        pt = JointTrajectoryPoint()
        pt.positions = [0.0, pitch]
        pt.time_from_start.sec = 2
        goal.trajectory.points.append(pt)
        self._pending_action = f'head:{mode}'
        fut = self.head_client.send_goal_async(goal)
        fut.add_done_callback(lambda f: self._on_head_goal_response(f, pitch, mode))

    def _on_head_goal_response(self, future, pitch, mode):
        gh = future.result()
        if gh is None or not gh.accepted:
            self._pending_action = None
            return
        gh.get_result_async().add_done_callback(
            lambda f: self._on_head_goal_result(f, pitch, mode))

    def _on_head_goal_result(self, future, pitch, mode):
        self.head_mode = mode
        self._pending_action = None

    def _trigger_localize(self):
        if self.localize_triggered:
            return
        if self.relocalize_cli.wait_for_service(timeout_sec=3.0):
            self.relocalize_cli.call_async(Empty.Request())
            self.localize_triggered = True
            self.get_logger().info('AMCL global localization triggered.')

    def _spin_localize(self):
        now = time.time()
        dt = (now - self._last_localize_tick) if self._last_localize_tick is not None else 0.0
        self._last_localize_tick = now

        if self._localize_nudge_until is not None:
            if now < self._localize_nudge_until:
                t = Twist()
                t.linear.x = self._localize_nudge_dir * self.LOCALIZE_NUDGE_SPEED
                self.cmd_vel_pub.publish(t)
                return
            self._localize_nudge_until = None
            self._stop()
            return

        self.rotation_accumulated += abs(self.LOCALIZE_SPIN_SPEED) * dt
        completed = int(self.rotation_accumulated // (2 * math.pi))
        if completed > self._localize_rotations_done:
            self._localize_rotations_done = completed
            self._localize_nudge_dir = 1 if (completed % 2 == 1) else -1
            self._localize_nudge_until = now + self.LOCALIZE_NUDGE_DURATION
            self._stop()
            return

        t = Twist()
        t.angular.z = self.LOCALIZE_SPIN_SPEED
        self.cmd_vel_pub.publish(t)

    def _send_goal(self, pose: PoseStamped):
        goal_msg = NavigateToPose.Goal()
        pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose = pose
        self.busy = True
        self.get_logger().info(
            f'Nav2 goal ({self.current_goal_id}): '
            f'({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})')
        fut = self.nav_client.send_goal_async(goal_msg)
        fut.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        gh = future.result()
        if gh is None or not gh.accepted:
            self.busy = False
            self.spin_started_at = self.get_clock().now()
            return
        self._current_goal_handle = gh
        gh.get_result_async().add_done_callback(self._on_goal_result)

    def _cancel_goal(self):
        if self._current_goal_handle:
            self._current_goal_handle.cancel_goal_async()

    def _on_goal_result(self, future):
        self.busy = False
        status = future.result().status
        gid = self.current_goal_id
        self.current_goal_id = None

        if gid == 'pick' and status == 4:
            self.get_logger().info('Pick station reached via Nav2.')
            self._consecutive_emergencies = 0
            self.spin_started_at = self.get_clock().now()
            self.patrol_index = 0
            self.patrol_targets = []
            self.state = 'DISCOVER_PLACE'

        elif gid == 'place' and status == 4:
            self.get_logger().info('Place station reached via Nav2.')
            self.state = 'DONE'

        elif status == 4:
            self._consecutive_emergencies = 0
            self.spin_started_at = self.get_clock().now()

        else:
            self.get_logger().warn(f'Goal "{gid}" ended status={status}. Retrying.')
            self.spin_started_at = self.get_clock().now()

    def _spin(self):
        t = Twist()
        t.angular.z = self.SEARCH_SPEED
        self.cmd_vel_pub.publish(t)

    def _stop(self):
        self.cmd_vel_pub.publish(Twist())

    def _rotate(self, vel, duration):
        ramp = min(0.3, duration / 2.0)
        start = time.time()
        end = start + duration
        while time.time() < end:
            now = time.time()
            elapsed = now - start
            remaining = end - now
            if elapsed < ramp:
                scale = elapsed / ramp
            elif remaining < ramp:
                scale = remaining / ramp
            else:
                scale = 1.0
            t = Twist()
            t.angular.z = vel * scale
            self.cmd_vel_pub.publish(t)
            time.sleep(0.1)
        self._stop()

    def _backup(self):
        self._rotate(0.0, self.BACKUP_DURATION)
        t = Twist()
        t.linear.x = -0.15
        for _ in range(int(self.BACKUP_DURATION * 10)):
            self.cmd_vel_pub.publish(t)
            time.sleep(0.1)
        self._stop()

    def _sector_min_range(self, center_deg, half_width_deg) -> float:
        if self._last_scan is None:
            return float('inf')
        msg = self._last_scan
        center = math.radians(center_deg)
        half = math.radians(half_width_deg)
        best = float('inf')
        for i in range(len(msg.ranges)):
            r = msg.ranges[i]
            if not (msg.range_min < r < 10.0):
                continue
            angle = msg.angle_min + i * msg.angle_increment
            diff = math.atan2(math.sin(angle - center), math.cos(angle - center))
            if abs(diff) <= half:
                best = min(best, r)
        return best

    def _find_best_escape_heading(self):
        step = 360.0 / self.ESCAPE_SECTOR_COUNT
        best_angle, best_range = 0.0, -1.0
        for k in range(self.ESCAPE_SECTOR_COUNT):
            center = -180.0 + k * step
            r = self._sector_min_range(center, step / 2.0)
            if r > best_range:
                best_range, best_angle = r, center
        return best_angle, best_range

    def _recover_from_obstacle(self, escalated: bool):
        front = self._sector_min_range(0.0, self.FRONT_CONE_DEG)
        rear = self._sector_min_range(180.0, 40.0)

        if rear >= self.ESCAPE_REAR_CLEAR_MIN:
            self._backup()
            if escalated:
                self._rotate(random.choice([1, -1]) * 0.5, self.RECOVERY_ROTATE_DURATION)
            return

        self.get_logger().warn('Wedged. Scanning for clearest heading.')
        if self._last_amcl_xy is not None:
            self._danger_zones.append(self._last_amcl_xy)
            if len(self._danger_zones) > 20:
                self._danger_zones = self._danger_zones[-20:]

        best_angle, best_range = self._find_best_escape_heading()
        turn_dir = 1 if best_angle > 0 else -1
        duration = min(abs(math.radians(best_angle)) / self.ESCAPE_ROTATE_SPEED,
                       self.ESCAPE_MAX_ROTATE_DURATION)
        if escalated:
            duration *= 1.5
        self._rotate(turn_dir * self.ESCAPE_ROTATE_SPEED, duration)

        front_after = self._sector_min_range(0.0, self.FRONT_CONE_DEG)
        if front_after >= self.ESCAPE_REAR_CLEAR_MIN and self.state != 'LOCALIZE':
            t = Twist()
            t.linear.x = self.ESCAPE_FORWARD_SPEED
            for _ in range(int(self.ESCAPE_FORWARD_DURATION * 10)):
                self.cmd_vel_pub.publish(t)
                time.sleep(0.1)
            self._stop()

    def _elapsed(self) -> float:
        if self.spin_started_at is None:
            return 0.0
        return (self.get_clock().now() - self.spin_started_at).nanoseconds * 1e-9

    def _save_yaml(self):
        import yaml
        import os
        data = {}
        for mid, pose in self.approach_poses.items():
            data[f'marker_{mid}'] = {
                'x': float(pose.pose.position.x),
                'y': float(pose.pose.position.y),
                'z': float(pose.pose.position.z),
                'qx': float(pose.pose.orientation.x),
                'qy': float(pose.pose.orientation.y),
                'qz': float(pose.pose.orientation.z),
                'qw': float(pose.pose.orientation.w),
            }
        path = os.path.expanduser('/home/osama/tiago_ws/found_markers.yaml')
        with open(path, 'w') as f:
            yaml.dump(data, f, default_flow_style=False)
        self.get_logger().info(f'Saved: {path}')


def main():
    rclpy.init()
    node = ArucoNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
