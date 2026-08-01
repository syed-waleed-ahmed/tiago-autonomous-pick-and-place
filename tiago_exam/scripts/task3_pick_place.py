#!/usr/bin/env python3
"""
Task 3: Pick and Place - Adaptive Arm Control
Flow: INIT -> LOCALIZE -> GOTO_PICK -> DISCOVER_CUBE -> GRASP -> GOTO_PLACE -> PLACE -> (repeat) -> DONE
"""
import math
import os
import time
import threading

import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from std_srvs.srv import Empty
from linkattacher_msgs.srv import AttachLink, DetachLink
from aruco_msgs.msg import MarkerArray

import PyKDL as kdl


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


class TaskThreePickPlace(Node):

    # ================================================================
    # CONFIGURATION
    # ================================================================
    ARM_FOLD = [2.6, -1.5, 0.6, 2.0, 1.2, -1.39, 2.0]
    BASE_REACH = [1.44, 0.50, -0.40, 1.20, 0.60, -0.30, 2.00]
    PLACE_LOWER = -0.10

    ARM_JOINT_NAMES = [
        'arm_1_joint', 'arm_2_joint', 'arm_3_joint', 'arm_4_joint',
        'arm_5_joint', 'arm_6_joint', 'arm_7_joint',
    ]
    GRIPPER_JOINT_NAMES = ['gripper_left_finger_joint', 'gripper_right_finger_joint']
    TORSO_JOINT_NAME = 'torso_lift_joint'

    TORSO_PICK_HEIGHT = 0.20
    TORSO_NAV_HEIGHT  = 0.10

    GRIPPER_OPEN   = 0.044
    GRIPPER_CLOSED = 0.030

    PICK_ID       = 26
    PLACE_ID      = 238
    CUBE_SEQUENCE = [63, 582]

    AMCL_COV_XY  = 0.07
    AMCL_COV_YAW = 0.15
    LOCALIZE_SPIN_SPEED = 0.4
    LOCALIZE_MAX_DURATION = 150.0
    COV_CHECK_INTERVAL = 0.5

    CUBE_SEARCH_TIMEOUT      = 20.0
    CUBE_MARKER_MIN_RANGE    = 0.15
    CUBE_MARKER_MAX_RANGE    = 2.0
    CUBE_CONSISTENCY_COUNT   = 2
    CUBE_CONSISTENCY_RADIUS  = 0.05

    HEAD_PITCH_NAV    = -0.35
    HEAD_PITCH_CUBE   = -0.55
    HEAD_PITCH_PLACE  = -0.60

    MAX_RETRIES = 3

    def __init__(self):
        super().__init__('task3_pick_place')

        self.lock = threading.Lock()
        cb_group = ReentrantCallbackGroup()

        # ---- state ----
        self.state = 'INIT'
        self.cube_index = 0
        self.station_frames = {}
        self.station_approach_poses = {}
        self._last_amcl_xy = None
        self.amcl_cov = None
        self.amcl_converged = False
        self.localize_triggered = False
        self.head_mode = None
        self.cube_pose_base = None
        self.cube_detection_history = []
        self._last_cov_display_time = 0
        self.cube_detected = False
        self.target_cube_id = 63
        self._rotation_done = False

        # ---- publishers/subscribers ----
        self.cmd_vel_pub = self.create_publisher(Twist, '/mobile_base_controller/cmd_vel_unstamped', 10)

        map_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=QoSReliabilityPolicy.RELIABLE)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self._on_amcl, 10,
                                 callback_group=cb_group)
        self.create_subscription(MarkerArray, '/aruco_cubes/markers', self._on_cube_markers, 10,
                                 callback_group=cb_group)

        # ---- action/service clients ----
        self.arm_client = ActionClient(self, FollowJointTrajectory,
                                       'arm_controller/follow_joint_trajectory',
                                       callback_group=cb_group)
        self.torso_client = ActionClient(self, FollowJointTrajectory,
                                         'torso_controller/follow_joint_trajectory',
                                         callback_group=cb_group)
        self.gripper_client = ActionClient(self, FollowJointTrajectory,
                                           'gripper_controller/follow_joint_trajectory',
                                           callback_group=cb_group)
        self.head_client = ActionClient(self, FollowJointTrajectory,
                                        'head_controller/follow_joint_trajectory',
                                        callback_group=cb_group)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose',
                                       callback_group=cb_group)
        self.relocalize_cli = self.create_client(Empty, '/reinitialize_global_localization',
                                                 callback_group=cb_group)
        self.attach_cli = self.create_client(AttachLink, '/ATTACHLINK', callback_group=cb_group)
        self.detach_cli = self.create_client(DetachLink, '/DETACHLINK', callback_group=cb_group)

        self.get_logger().info('Task 3 pick-and-place node constructed.')

    # ================================================================
    # ADAPTIVE ARM POSITIONING
    # ================================================================

    def _calculate_arm_position_from_cube(self) -> list:
        if self.cube_pose_base is None:
            return self.BASE_REACH.copy()

        cx, cy, cz = self.cube_pose_base
        arm_pos = self.BASE_REACH.copy()

        arm_pos[0] = max(0.8, min(2.0, 1.0 + (cx - 0.3) * 1.2))
        arm_pos[1] = max(-0.5, min(0.9, 0.3 + cy * 1.2))
        arm_pos[2] = max(-0.6, min(-0.1, -0.4 + cz * 0.3))
        arm_pos[4] = max(0.3, min(0.7, 0.5 + cz * 0.2))
        arm_pos[3] = 1.2
        arm_pos[5] = -0.3
        arm_pos[6] = 2.0
        return arm_pos

    def _get_adaptive_reach_position(self) -> list:
        if self.cube_pose_base is not None:
            return self._calculate_arm_position_from_cube()
        return self.BASE_REACH.copy()

    # ================================================================
    # HELPERS
    # ================================================================

    def _wait_future(self, future, timeout) -> bool:
        start = time.time()
        while not future.done():
            if time.time() - start > timeout:
                return False
            time.sleep(0.02)
        return True

    def _retry(self, fn, name, max_retries=None) -> bool:
        max_retries = max_retries or self.MAX_RETRIES
        for attempt in range(1, max_retries + 1):
            if fn():
                return True
            self.get_logger().warn(f'{name} failed (attempt {attempt}/{max_retries}) - retrying...')
            time.sleep(0.5)
        self.get_logger().error(f'{name} failed after {max_retries} attempts.')
        return False

    # ================================================================
    # MOTION PRIMITIVES
    # ================================================================

    def move_arm(self, positions, motion_name="", timeout=10.0) -> bool:
        if not self.arm_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('Arm action server not available')
            return False

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = self.ARM_JOINT_NAMES
        pt = JointTrajectoryPoint()
        pt.positions = list(positions)
        pt.time_from_start.sec = 3
        goal.trajectory.points.append(pt)

        if motion_name:
            self.get_logger().info(f'Arm: {motion_name}...')

        fut = self.arm_client.send_goal_async(goal)
        if not self._wait_future(fut, timeout):
            self.get_logger().error(f'Arm goal send timed out ({motion_name})')
            return False
        gh = fut.result()
        if gh is None or not gh.accepted:
            self.get_logger().error(f'Arm goal rejected ({motion_name})')
            return False

        res_fut = gh.get_result_async()
        if not self._wait_future(res_fut, timeout):
            self.get_logger().error(f'Arm execution timed out ({motion_name})')
            return False

        result = res_fut.result()
        ok = result is not None and result.result.error_code == 0
        if ok and motion_name:
            self.get_logger().info(f'Arm: {motion_name} complete.')
        return ok

    def move_torso(self, height, motion_name="", timeout=10.0) -> bool:
        if not self.torso_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('Torso action server not available')
            return False

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = [self.TORSO_JOINT_NAME]
        pt = JointTrajectoryPoint()
        pt.positions = [height]
        pt.time_from_start.sec = 3
        goal.trajectory.points.append(pt)

        if motion_name:
            self.get_logger().info(f'Torso: {motion_name}...')

        fut = self.torso_client.send_goal_async(goal)
        if not self._wait_future(fut, timeout):
            self.get_logger().error(f'Torso goal send timed out ({motion_name})')
            return False
        gh = fut.result()
        if gh is None or not gh.accepted:
            self.get_logger().error(f'Torso goal rejected ({motion_name})')
            return False

        res_fut = gh.get_result_async()
        if not self._wait_future(res_fut, timeout):
            self.get_logger().error(f'Torso execution timed out ({motion_name})')
            return False

        result = res_fut.result()
        ok = result is not None and result.result.error_code == 0
        if ok and motion_name:
            self.get_logger().info(f'Torso: {motion_name} complete.')
        return ok

    def set_gripper(self, position, motion_name="", timeout=3.0) -> bool:
        if not self.gripper_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('Gripper action server not available.')
            return False

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = self.GRIPPER_JOINT_NAMES
        pt = JointTrajectoryPoint()
        pt.positions = [position, position]
        pt.time_from_start.sec = 1
        goal.trajectory.points.append(pt)

        if motion_name:
            self.get_logger().info(f'Gripper: {motion_name}...')

        fut = self.gripper_client.send_goal_async(goal)
        if not self._wait_future(fut, timeout):
            self.get_logger().error(f'Gripper goal send timed out ({motion_name})')
            return False
        gh = fut.result()
        if gh is None or not gh.accepted:
            self.get_logger().error(f'Gripper goal rejected ({motion_name})')
            return False

        res_fut = gh.get_result_async()
        if not self._wait_future(res_fut, timeout):
            self.get_logger().error(f'Gripper execution timed out ({motion_name})')
            return False

        if motion_name:
            self.get_logger().info(f'Gripper: {motion_name} complete.')
        return True

    def attach_cube(self, cube_id) -> bool:
        if not self.attach_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('ATTACHLINK unavailable - relying on friction grasp.')
            return False

        req = AttachLink.Request()
        req.model1_name = 'tiago'
        req.link1_name = 'gripper_left_finger_link'
        req.model2_name = f'aruco_cube_exam_id{cube_id}'
        req.link2_name = 'link'

        fut = self.attach_cli.call_async(req)
        if not self._wait_future(fut, 5.0):
            self.get_logger().warn(f'Attach call timed out for cube {cube_id}.')
            return False
        if fut.result() is None:
            self.get_logger().warn(f'Attach call returned no result for cube {cube_id}.')
            return False
        self.get_logger().info(f'Cube {cube_id} attached.')
        return True

    def detach_cube(self, cube_id) -> bool:
        if not self.detach_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('DETACHLINK unavailable.')
            return False

        req = DetachLink.Request()
        req.model1_name = 'tiago'
        req.link1_name = 'gripper_left_finger_link'
        req.model2_name = f'aruco_cube_exam_id{cube_id}'
        req.link2_name = 'link'

        fut = self.detach_cli.call_async(req)
        if not self._wait_future(fut, 5.0):
            self.get_logger().warn(f'Detach call timed out for cube {cube_id}.')
            return False
        if fut.result() is None:
            self.get_logger().warn(f'Detach call returned no result for cube {cube_id}.')
            return False
        self.get_logger().info(f'Cube {cube_id} detached.')
        return True

    def _tilt_head(self, pitch, mode, timeout=3.0) -> bool:
        if self.head_mode == mode:
            return True
        if not self.head_client.wait_for_server(timeout_sec=1.0):
            return False
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = ['head_1_joint', 'head_2_joint']
        pt = JointTrajectoryPoint()
        pt.positions = [0.0, pitch]
        pt.time_from_start.sec = 2
        goal.trajectory.points.append(pt)
        fut = self.head_client.send_goal_async(goal)
        if not self._wait_future(fut, timeout):
            return False
        gh = fut.result()
        if gh is None or not gh.accepted:
            return False
        res_fut = gh.get_result_async()
        if not self._wait_future(res_fut, timeout):
            return False
        self.head_mode = mode
        return True

    # ================================================================
    # ROTATION
    # ================================================================

    def _rotate_left_120(self):
        self.get_logger().info('Rotating 120 degrees left...')
        t = Twist()
        t.angular.z = 0.6
        duration = (2 * math.pi / 3) / 0.6
        start_time = time.time()
        while time.time() - start_time < duration:
            self.cmd_vel_pub.publish(t)
            time.sleep(0.05)
        self._stop()
        time.sleep(0.3)

    # ================================================================
    # PICK SEQUENCE
    # ================================================================

    def _pick_cube(self) -> bool:
        cube_id = self.CUBE_SEQUENCE[self.cube_index]
        self.get_logger().info(f'--- Picking cube {cube_id} ---')

        # 1. Raise torso
        if not self._retry(lambda: self.move_torso(self.TORSO_PICK_HEIGHT, 'Raising torso'), 'raise torso'):
            self.get_logger().warn('Torso movement failed - continuing with pick sequence anyway.')

        # 2. Open gripper
        if not self._retry(lambda: self.set_gripper(self.GRIPPER_OPEN, 'Opening gripper'), 'open gripper'):
            return False

        # 3. Move to reach position (above cube)
        reach_pos = self._get_adaptive_reach_position()
        if not self._retry(lambda: self.move_arm(reach_pos, 'Adaptive Reach'), 'adaptive reach'):
            return False

        # 4. Move to grasp position (at cube)
        grasp_pos = [reach_pos[0], reach_pos[1], reach_pos[2] - 0.05, reach_pos[3], reach_pos[4], reach_pos[5], reach_pos[6]]
        if not self._retry(lambda: self.move_arm(grasp_pos, 'Descend to grasp'), 'descend'):
            return False

        # 5. Close gripper
        if not self._retry(lambda: self.set_gripper(self.GRIPPER_CLOSED, 'Closing gripper'), 'close gripper'):
            return False

        # 6. Attach cube
        if not self.attach_cube(cube_id):
            self.get_logger().warn('Attach failed - relying on friction grasp only.')

        # 7. Fold arm
        if not self._retry(lambda: self.move_arm(self.ARM_FOLD, 'Folding arm'), 'fold arm'):
            return False

        # 8. Lower torso
        if not self._retry(lambda: self.move_torso(self.TORSO_NAV_HEIGHT, 'Lowering torso'), 'lower torso'):
            self.get_logger().warn('Torso lowering failed - continuing with pick sequence anyway.')

        return True

    # ================================================================
    # PLACE SEQUENCE
    # ================================================================

    def _place_cube(self) -> bool:
        cube_id = self.CUBE_SEQUENCE[self.cube_index]
        self.get_logger().info(f'--- Placing cube {cube_id} ---')

        # 1. Raise torso
        if not self._retry(lambda: self.move_torso(self.TORSO_PICK_HEIGHT, 'Raising torso for place'), 'raise torso'):
            self.get_logger().warn('Torso movement failed - continuing with place sequence anyway.')

        # 2. Move to reach position (above table)
        reach_pos = self.BASE_REACH.copy()
        if not self._retry(lambda: self.move_arm(reach_pos, 'Place Reach'), 'place reach'):
            return False

        # 3. Lower arm to place position
        place_pos = reach_pos.copy()
        place_pos[2] += self.PLACE_LOWER
        if not self._retry(lambda: self.move_arm(place_pos, 'Descend to place'), 'descend place'):
            return False

        # 4. Raise torso to 0.30
        if not self._retry(lambda: self.move_torso(0.30, 'Raising torso to 0.30'), 'raise torso 0.30'):
            self.get_logger().warn('Torso raise to 0.30 failed - continuing anyway.')

        # 5. Rotate 120° left
        self._rotate_left_120()

        # 6. Open gripper
        if not self._retry(lambda: self.set_gripper(self.GRIPPER_OPEN, 'Opening gripper'), 'open gripper'):
            return False

        # 7. Detach cube
        if not self.detach_cube(cube_id):
            self.get_logger().warn('Detach failed - continuing anyway.')

        # 8. Lift arm
        lift_pos = place_pos.copy()
        lift_pos[2] += 0.10
        if not self._retry(lambda: self.move_arm(lift_pos, 'Lifting arm'), 'lift arm'):
            return False

        # 9. Fold arm
        if not self._retry(lambda: self.move_arm(self.ARM_FOLD, 'Folding arm'), 'fold arm'):
            return False

        # 10. Lower torso
        if not self._retry(lambda: self.move_torso(self.TORSO_NAV_HEIGHT, 'Lowering torso'), 'lower torso'):
            self.get_logger().warn('Torso lowering failed - continuing with place sequence anyway.')

        return True

    # ================================================================
    # SUBSCRIBERS
    # ================================================================

    def _on_amcl(self, msg: PoseWithCovarianceStamped):
        c = msg.pose.covariance
        self.amcl_cov = (c[0], c[7], c[35])
        self._last_amcl_xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)

        if not self.amcl_converged:
            cx, cy, cyaw = self.amcl_cov
            now = time.time()
            if now - self._last_cov_display_time > self.COV_CHECK_INTERVAL:
                self.get_logger().info(
                    f'AMCL cov x={cx:.4f} y={cy:.4f} yaw={cyaw:.4f}  '
                    f'(threshold: {self.AMCL_COV_XY:.2f}, {self.AMCL_COV_YAW:.2f})'
                )
                self._last_cov_display_time = now

            if cx < self.AMCL_COV_XY and cy < self.AMCL_COV_XY and cyaw < self.AMCL_COV_YAW:
                self.amcl_converged = True
                self.get_logger().info(f'AMCL converged! cov=({cx:.4f}, {cy:.4f}, {cyaw:.4f})')

    def _on_cube_markers(self, msg: MarkerArray):
        if not msg.markers or self.state != 'DISCOVER_CUBE':
            return
        
        target_id = self.CUBE_SEQUENCE[self.cube_index]
        
        with self.lock:
            for m in msg.markers:
                if m.id != target_id:
                    continue
                    
                px, py, pz = (m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z)
                planar_range = math.hypot(px, py)
                
                if not (self.CUBE_MARKER_MIN_RANGE < planar_range < self.CUBE_MARKER_MAX_RANGE):
                    continue
                
                self.cube_detection_history.append((px, py, pz, time.time()))
                if len(self.cube_detection_history) > self.CUBE_CONSISTENCY_COUNT:
                    self.cube_detection_history.pop(0)
                
                if len(self.cube_detection_history) < self.CUBE_CONSISTENCY_COUNT:
                    continue
                
                avg_x = sum(p[0] for p in self.cube_detection_history) / len(self.cube_detection_history)
                avg_y = sum(p[1] for p in self.cube_detection_history) / len(self.cube_detection_history)
                avg_z = sum(p[2] for p in self.cube_detection_history) / len(self.cube_detection_history)
                
                if any(math.hypot(p[0] - avg_x, p[1] - avg_y) > self.CUBE_CONSISTENCY_RADIUS
                       for p in self.cube_detection_history):
                    self.cube_detection_history.clear()
                    continue
                
                self.cube_pose_base = (avg_x, avg_y, avg_z)
                self.cube_detected = True
                self.get_logger().info(f'Target cube {m.id} detected at: ({avg_x:.2f}, {avg_y:.2f}, {avg_z:.2f})')

    # ================================================================
    # NAVIGATION & LOCALIZATION
    # ================================================================

    def _load_saved_coordinates(self) -> bool:
        path = os.path.expanduser('~/tiago_ws/found_markers.yaml')
        if not os.path.exists(path):
            self.get_logger().error(f'Saved coordinates file not found: {path}')
            return False
        try:
            with open(path) as f:
                data = yaml.safe_load(f) or {}
        except Exception as e:
            self.get_logger().error(f'Failed to load {path}: {e}')
            return False

        loaded = 0
        for sid, name in ((self.PICK_ID, 'pick'), (self.PLACE_ID, 'place')):
            key = f'marker_{sid}'
            if key not in data:
                self.get_logger().error(f'Marker {sid} not found in saved file')
                continue
            d = data[key]
            gx, gy, qz, qw = d['x'], d['y'], d['qz'], d['qw']
            yaw = 2.0 * math.atan2(qz, qw)

            self.station_frames[sid] = kdl.Frame(kdl.Rotation.Identity(), kdl.Vector(gx, gy, 0.0))
            approach = kdl.Frame(kdl.Rotation.RotZ(yaw), kdl.Vector(gx, gy, 0.0))
            self.station_approach_poses[sid] = kdl_to_pose_stamped(
                approach, 'map', self.get_clock().now().to_msg())
            self.get_logger().info(f'Loaded {name} station marker {sid} at ({gx:.2f}, {gy:.2f})')
            loaded += 1
        return loaded == 2

    def _navigate_to(self, pose: PoseStamped, timeout=90.0) -> bool:
        if not self.nav_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error('NavigateToPose action server not available')
            return False
        goal = NavigateToPose.Goal()
        pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose = pose

        self.get_logger().info(f'Navigating to ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})...')
        fut = self.nav_client.send_goal_async(goal)
        if not self._wait_future(fut, 5.0):
            self.get_logger().error('Nav goal send timed out')
            return False
        gh = fut.result()
        if gh is None or not gh.accepted:
            self.get_logger().error('Nav goal rejected')
            return False

        res_fut = gh.get_result_async()
        if not self._wait_future(res_fut, timeout):
            self.get_logger().error('Navigation timed out - cancelling')
            gh.cancel_goal_async()
            return False

        status = res_fut.result().status
        if status == 4:
            self.get_logger().info('Navigation succeeded.')
            return True
        self.get_logger().warn(f'Navigation ended with status={status}.')
        return False

    def _trigger_localize(self):
        if self.localize_triggered:
            return
        if self.relocalize_cli.wait_for_service(timeout_sec=3.0):
            self.relocalize_cli.call_async(Empty.Request())
            self.localize_triggered = True
            self.get_logger().info('AMCL global localization triggered.')

    def _localize_blocking(self) -> bool:
        self._trigger_localize()
        self.get_logger().info('=== LOCALIZING: Spinning to converge ===')
        self._last_cov_display_time = 0

        start = time.time()
        while time.time() - start < self.LOCALIZE_MAX_DURATION:
            if self.amcl_converged:
                self._stop()
                self.get_logger().info('Localization complete!')
                return True
            t = Twist()
            t.angular.z = self.LOCALIZE_SPIN_SPEED
            self.cmd_vel_pub.publish(t)
            time.sleep(0.1)
        self._stop()
        self.get_logger().warn('AMCL did not converge within timeout - proceeding anyway.')
        return False

    # ================================================================
    # CUBE DISCOVERY
    # ================================================================

    def _discover_cube_blocking(self) -> bool:
        target_id = self.CUBE_SEQUENCE[self.cube_index]
        self.cube_pose_base = None
        self.cube_detection_history = []
        self.cube_detected = False
        
        self.get_logger().info(f'=== SEARCHING FOR CUBE {target_id} ===')
        
        if not self._tilt_head(self.HEAD_PITCH_CUBE, 'cube'):
            self.get_logger().warn('Could not tilt head for cube search')

        start = time.time()
        sweep_direction = 1
        last_switch_time = start
        detection_attempts = 0
        
        while time.time() - start < self.CUBE_SEARCH_TIMEOUT:
            if self.cube_pose_base is not None:
                self._stop()
                self.get_logger().info(f'✓ Target cube {target_id} detected at: {self.cube_pose_base}')
                return True
            
            elapsed = time.time() - start
            if elapsed - last_switch_time > 2.0:
                sweep_direction *= -1
                last_switch_time = elapsed
                detection_attempts += 1
                self.get_logger().info(f'Sweeping {"RIGHT" if sweep_direction > 0 else "LEFT"} for cube {target_id} (Attempt {detection_attempts})...')
            
            t = Twist()
            t.angular.z = 0.35 * sweep_direction
            self.cmd_vel_pub.publish(t)
            time.sleep(0.1)

        self._stop()
        self.get_logger().warn(f'✗ Cube {target_id} not found within {self.CUBE_SEARCH_TIMEOUT}s timeout.')
        return False

    def _stop(self):
        self.cmd_vel_pub.publish(Twist())

    # ================================================================
    # STATE MACHINE
    # ================================================================

    def run(self):
        self.get_logger().info('=== Task 3: Pick and Place - starting state machine ===')
        self.state = 'INIT'

        while rclpy.ok():
            self.get_logger().info(f'--- STATE: {self.state} ---')

            if self.state == 'INIT':
                if not self._retry(lambda: self.move_arm(self.ARM_FOLD, 'Fold to safe position'), 'fold arm'):
                    self.state = 'DONE'
                    continue
                self._tilt_head(self.HEAD_PITCH_NAV, 'nav')
                if not self._load_saved_coordinates():
                    self.get_logger().error('Failed to load saved coordinates. Aborting.')
                    self.state = 'DONE'
                    continue
                self.state = 'LOCALIZE'

            elif self.state == 'LOCALIZE':
                self._localize_blocking()
                self.state = 'GOTO_PICK'

            elif self.state == 'GOTO_PICK':
                self._tilt_head(self.HEAD_PITCH_NAV, 'nav')
                if not self._retry(lambda: self._navigate_to(self.station_approach_poses[self.PICK_ID]),
                                   'navigate to pick station'):
                    self.state = 'DONE'
                    continue
                self.state = 'DISCOVER_CUBE'

            elif self.state == 'DISCOVER_CUBE':
                if not self._discover_cube_blocking():
                    self.get_logger().error('Could not find target cube - aborting task.')
                    self.state = 'DONE'
                    continue
                self.state = 'GRASP'

            elif self.state == 'GRASP':
                if not self._pick_cube():
                    self.get_logger().error('Pick sequence failed - aborting task.')
                    self.state = 'DONE'
                    continue
                self.state = 'GOTO_PLACE'

            elif self.state == 'GOTO_PLACE':
                self._tilt_head(self.HEAD_PITCH_NAV, 'nav')
                if not self._retry(lambda: self._navigate_to(self.station_approach_poses[self.PLACE_ID]),
                                   'navigate to place station'):
                    self.state = 'DONE'
                    continue
                self.state = 'PLACE'

            elif self.state == 'PLACE':
                if not self._place_cube():
                    self.get_logger().error('Place sequence failed - aborting task.')
                    self.state = 'DONE'
                    continue

                self.cube_index += 1
                if self.cube_index >= len(self.CUBE_SEQUENCE):
                    self.state = 'DONE'
                    self.get_logger().info('✓ All cubes placed successfully!')
                else:
                    self.get_logger().info(f'--- Moving to next cube: {self.CUBE_SEQUENCE[self.cube_index]} ---')
                    self.state = 'GOTO_PICK'

            elif self.state == 'DONE':
                self.get_logger().info('========================================')
                self.get_logger().info('TASK 3 COMPLETE.')
                self.get_logger().info('========================================')
                return

            else:
                self.get_logger().error(f'Unknown state: {self.state}')
                return


def main():
    rclpy.init()
    node = TaskThreePickPlace()

    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    time.sleep(1.0)

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
