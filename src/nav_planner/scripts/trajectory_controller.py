#!/usr/bin/env python3

import math
import bisect
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy
)

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory


class TrajectoryTracker(Node):

    def __init__(self):

        super().__init__('trajectory_tracker')

        # ====================================================
        # CONFIGURATION
        # ====================================================

        self.declare_parameter('trajectory_topic',
                               '/trajectory')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('control_rate', 20.0)

        self.declare_parameter('kx', 0.8)
        self.declare_parameter('ky', 2.0)
        self.declare_parameter('ktheta', 2.0)

        self.declare_parameter('max_linear', 0.5)
        self.declare_parameter('max_angular', 1.0)

        self.declare_parameter('position_tolerance', 0.08)
        self.declare_parameter('yaw_tolerance', 0.15)
        self.declare_parameter('odom_timeout', 0.5)

        self.trajectory_topic = self.get_parameter(
            'trajectory_topic').value
        self.odom_topic = self.get_parameter(
            'odom_topic').value
        self.cmd_vel_topic = self.get_parameter(
            'cmd_vel_topic').value

        self.control_rate = float(
            self.get_parameter('control_rate').value)

        self.kx = float(self.get_parameter('kx').value)
        self.ky = float(self.get_parameter('ky').value)
        self.ktheta = float(self.get_parameter('ktheta').value)

        self.max_linear = float(
            self.get_parameter('max_linear').value)
        self.max_angular = float(
            self.get_parameter('max_angular').value)

        self.position_tolerance = float(
            self.get_parameter('position_tolerance').value)
        self.yaw_tolerance = float(
            self.get_parameter('yaw_tolerance').value)
        self.odom_timeout = float(
            self.get_parameter('odom_timeout').value)

        if self.control_rate <= 0.0:
            raise ValueError('control_rate must be positive')

        # ====================================================
        # TRAJECTORY STATE
        # ====================================================

        self.trajectory = []
        self.times = []
        self.duration = 0.0

        self.trajectory_received = False
        self.trajectory_started = False
        self.start_time = None
        self.finished = False

        self.last_signature = None

        # ====================================================
        # ODOMETRY STATE
        # ====================================================

        self.x = None
        self.y = None
        self.yaw = None

        self.last_odom_time = None

        # ====================================================
        # DEBUG
        # ====================================================

        self.last_debug_time = 0.0

        # ====================================================
        # ROS QOS
        # ====================================================

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # ====================================================
        # SUBSCRIBERS
        # ====================================================

        self.trajectory_sub = self.create_subscription(
            JointTrajectory,
            self.trajectory_topic,
            self.trajectory_callback,
            qos
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10
        )

        # ====================================================
        # PUBLISHER
        # ====================================================

        self.cmd_pub = self.create_publisher(
            Twist,
            self.cmd_vel_topic,
            10
        )

        # ====================================================
        # CONTROL TIMER
        # ====================================================

        self.timer = self.create_timer(
            1.0 / self.control_rate,
            self.control_loop
        )

        self.get_logger().info(
            '============================================'
        )
        self.get_logger().info(
            'Trajectory tracker started'
        )
        self.get_logger().info(
            f'Subscribing: {self.trajectory_topic}'
        )
        self.get_logger().info(
            f'Odometry: {self.odom_topic}'
        )
        self.get_logger().info(
            f'Publishing: {self.cmd_vel_topic}'
        )
        self.get_logger().info(
            'Waiting for trajectory and odometry...'
        )

    # ========================================================
    # ANGLE UTILITIES
    # ========================================================

    @staticmethod
    def normalize_angle(angle):

        return math.atan2(
            math.sin(angle),
            math.cos(angle)
        )

    @staticmethod
    def quaternion_to_yaw(q):

        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

    # ========================================================
    # ODOM CALLBACK
    # ========================================================

    def odom_callback(self, msg):

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        self.yaw = self.quaternion_to_yaw(
            msg.pose.pose.orientation
        )

        self.last_odom_time = time.monotonic()

    # ========================================================
    # STOP ROBOT
    # ========================================================

    def publish_stop(self):

        cmd = Twist()

        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0

        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)

    # ========================================================
    # TRAJECTORY CALLBACK
    # ========================================================

    def trajectory_callback(self, msg):

        if len(msg.points) < 2:
            self.get_logger().warning(
                'Received trajectory with fewer than 2 points'
            )
            self.publish_stop()
            return

        # Ensure trajectory uses the expected variable order.
        # The trajectory generator publishes:
        # joint_names = ['x', 'y', 'yaw']
        if msg.joint_names != ['x', 'y', 'yaw']:
            self.get_logger().error(
                f'Unexpected trajectory joint_names: '
                f'{msg.joint_names}. Expected [x, y, yaw].'
            )
            self.publish_stop()
            return

        trajectory = []

        for i, point in enumerate(msg.points):

            if len(point.positions) < 3:
                self.get_logger().error(
                    f'Point {i} has fewer than 3 positions'
                )
                self.publish_stop()
                return

            if len(point.velocities) < 3:
                self.get_logger().error(
                    f'Point {i} has fewer than 3 velocities'
                )
                self.publish_stop()
                return

            t = (
                point.time_from_start.sec
                + point.time_from_start.nanosec * 1e-9
            )

            x = float(point.positions[0])
            y = float(point.positions[1])
            yaw = float(point.positions[2])

            vx = float(point.velocities[0])
            vy = float(point.velocities[1])
            omega = float(point.velocities[2])

            # Speed magnitude, as used by the original tracker
            v = math.hypot(vx, vy)

            values = [t, x, y, yaw, vx, vy, omega, v]

            if not all(math.isfinite(value) for value in values):
                self.get_logger().error(
                    f'Non-finite value in trajectory point {i}'
                )
                self.publish_stop()
                return

            trajectory.append({
                'time': t,
                'x': x,
                'y': y,
                'yaw': yaw,
                'v': v,
                'vx': vx,
                'vy': vy,
                'omega': omega
            })

        # Verify strictly increasing time
        for i in range(1, len(trajectory)):

            if trajectory[i]['time'] <= trajectory[i - 1]['time']:
                self.get_logger().error(
                    f'Trajectory time is not strictly increasing '
                    f'at point {i}'
                )
                self.publish_stop()
                return

        if trajectory[0]['time'] < -1e-9:
            self.get_logger().error(
                'Trajectory start time must be >= 0'
            )
            self.publish_stop()
            return

        # Prevent restarting on unchanged repeated messages
        signature = tuple(
            (
                p['time'], p['x'], p['y'],
                p['yaw'], p['vx'], p['vy'], p['omega']
            )
            for p in trajectory
        )

        if signature == self.last_signature:
            return

        self.last_signature = signature

        # Accept new trajectory and reset tracking clock
        self.trajectory = trajectory
        self.times = [
            p['time'] for p in self.trajectory
        ]

        self.duration = self.times[-1]

        self.trajectory_received = True
        self.trajectory_started = False
        self.start_time = None
        self.finished = False

        self.get_logger().info(
            '============================================'
        )
        self.get_logger().info(
            'New time-parameterized trajectory received'
        )
        self.get_logger().info(
            f'Points: {len(self.trajectory)}'
        )
        self.get_logger().info(
            f'Duration: {self.duration:.3f} s'
        )
        self.get_logger().info(
            f'Start: ({self.trajectory[0]["x"]:.3f}, '
            f'{self.trajectory[0]["y"]:.3f})'
        )
        self.get_logger().info(
            f'Goal: ({self.trajectory[-1]["x"]:.3f}, '
            f'{self.trajectory[-1]["y"]:.3f})'
        )

        self.publish_stop()

    # ========================================================
    # INTERPOLATE TRAJECTORY
    # ========================================================

    def interpolate_trajectory(self, elapsed):

        if elapsed <= self.times[0]:
            return self.trajectory[0]

        if elapsed >= self.times[-1]:
            return self.trajectory[-1]

        i = bisect.bisect_right(
            self.times,
            elapsed
        ) - 1

        p1 = self.trajectory[i]
        p2 = self.trajectory[i + 1]

        t1 = p1['time']
        t2 = p2['time']

        dt = t2 - t1

        if dt <= 0.0:
            return p1

        alpha = (elapsed - t1) / dt

        # Position
        x = p1['x'] + alpha * (p2['x'] - p1['x'])
        y = p1['y'] + alpha * (p2['y'] - p1['y'])

        # Shortest-angle yaw interpolation
        yaw_difference = self.normalize_angle(
            p2['yaw'] - p1['yaw']
        )

        yaw = self.normalize_angle(
            p1['yaw'] + alpha * yaw_difference
        )

        # Linear velocity interpolation
        vx = p1['vx'] + alpha * (p2['vx'] - p1['vx'])
        vy = p1['vy'] + alpha * (p2['vy'] - p1['vy'])

        v = math.hypot(vx, vy)

        # Angular velocity interpolation
        omega = (
            p1['omega']
            + alpha * (p2['omega'] - p1['omega'])
        )

        return {
            'time': elapsed,
            'x': x,
            'y': y,
            'yaw': yaw,
            'v': v,
            'vx': vx,
            'vy': vy,
            'omega': omega
        }

    # ========================================================
    # CONTROL LOOP
    # ========================================================

    def control_loop(self):

        # ----------------------------------------------------
        # No trajectory yet
        # ----------------------------------------------------

        if not self.trajectory_received:
            self.publish_stop()
            return

        # ----------------------------------------------------
        # Wait for odometry
        # ----------------------------------------------------

        if (
            self.x is None
            or self.y is None
            or self.yaw is None
            or self.last_odom_time is None
        ):
            self.publish_stop()
            return

        # ----------------------------------------------------
        # Odometry timeout
        # ----------------------------------------------------

        odom_age = time.monotonic() - self.last_odom_time

        if odom_age > self.odom_timeout:

            self.publish_stop()

            if not self.finished:
                self.finished = True
                self.get_logger().error(
                    'Odometry timeout. Trajectory aborted.'
                )

            return

        # ----------------------------------------------------
        # Start trajectory clock when odometry is available
        # ----------------------------------------------------

        if self.start_time is None:

            self.start_time = time.monotonic()
            self.trajectory_started = True

            self.get_logger().info(
                '============================================'
            )
            self.get_logger().info(
                'Trajectory tracking STARTED'
            )

        # ----------------------------------------------------
        # Trajectory time
        # ----------------------------------------------------

        elapsed = time.monotonic() - self.start_time

        # ----------------------------------------------------
        # Completion check
        # ----------------------------------------------------

        if elapsed >= self.duration:

            final = self.trajectory[-1]

            dx_final = final['x'] - self.x
            dy_final = final['y'] - self.y

            position_error = math.hypot(
                dx_final,
                dy_final
            )

            heading_error = abs(
                self.normalize_angle(
                    final['yaw'] - self.yaw
                )
            )

            self.publish_stop()
            self.finished = True

            self.get_logger().info(
                '============================================'
            )
            self.get_logger().info(
                'Trajectory COMPLETE'
            )
            self.get_logger().info(
                f'Final position error: {position_error:.3f} m'
            )
            self.get_logger().info(
                f'Final heading error: {heading_error:.3f} rad'
            )

            if (
                position_error > self.position_tolerance
                or heading_error > self.yaw_tolerance
            ):
                self.get_logger().warning(
                    'Trajectory time ended, but final pose is '
                    'outside the specified tolerance.'
                )

            return

        if self.finished:
            self.publish_stop()
            return

        # ----------------------------------------------------
        # Desired reference state
        # ----------------------------------------------------

        ref = self.interpolate_trajectory(elapsed)

        xd = ref['x']
        yd = ref['y']
        yawd = ref['yaw']

        vd = ref['v']
        omegad = ref['omega']

        # ----------------------------------------------------
        # Position error in world frame
        # ----------------------------------------------------

        dx = xd - self.x
        dy = yd - self.y

        # ----------------------------------------------------
        # Convert error to robot frame
        # ----------------------------------------------------

        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)

        ex = cos_yaw * dx + sin_yaw * dy
        ey = -sin_yaw * dx + cos_yaw * dy

        # ----------------------------------------------------
        # Heading error
        # ----------------------------------------------------

        etheta = self.normalize_angle(yawd - self.yaw)

        # ----------------------------------------------------
        # Feed-forward + feedback control
        # ----------------------------------------------------

        v_cmd = (
            vd * math.cos(etheta)
            + self.kx * ex
        )

        omega_cmd = (
            omegad
            + self.ky * vd * ey
            + self.ktheta * math.sin(etheta)
        )

        # ----------------------------------------------------
        # Command limits
        # ----------------------------------------------------

        v_cmd = max(
            -self.max_linear,
            min(self.max_linear, v_cmd)
        )

        omega_cmd = max(
            -self.max_angular,
            min(self.max_angular, omega_cmd)
        )

        # ----------------------------------------------------
        # Publish cmd_vel
        # ----------------------------------------------------

        cmd = Twist()

        cmd.linear.x = float(v_cmd)
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0

        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = float(omega_cmd)

        self.cmd_pub.publish(cmd)

        # ----------------------------------------------------
        # Debug output
        # ----------------------------------------------------

        now = time.monotonic()

        if now - self.last_debug_time > 1.0:

            self.last_debug_time = now

            position_error = math.hypot(dx, dy)

            self.get_logger().info(
                f't={elapsed:6.2f} | '
                f'actual=({self.x:7.3f}, {self.y:7.3f}) | '
                f'desired=({xd:7.3f}, {yd:7.3f}) | '
                f'ex={ex:6.3f} | '
                f'ey={ey:6.3f} | '
                f'v_ref={vd:5.3f} | '
                f'v_cmd={v_cmd:5.3f} | '
                f'w_ref={omegad:5.3f} | '
                f'w_cmd={omega_cmd:5.3f} | '
                f'err={position_error:.3f}'
            )

    # ========================================================
    # MAIN
    # ========================================================

def main(args=None):

    rclpy.init(args=args)

    node = None

    try:
        node = TrajectoryTracker()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    except Exception as e:
        print(f'ERROR: {e}')

    finally:
        if node is not None:
            node.publish_stop()
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
