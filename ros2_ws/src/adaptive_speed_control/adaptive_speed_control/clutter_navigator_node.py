#!/usr/bin/env python3
"""
Clutter Navigator Node for Autonomous Exploration in Cluttered Environments.
Generates reactive driving commands using sector gap analysis and potential field repulsion.
Supports comparative modes: 'adaptive' (routes through adaptive speed controller) and 'fixed' (constant speed).
"""

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import TwistStamped, Twist
from sensor_msgs.msg import LaserScan


class ClutterNavigator(Node):
    def __init__(self):
        super().__init__('clutter_navigator')

        # Parameters
        self.declare_parameter('nominal_speed', 0.45)        # m/s cruise speed in open space
        self.declare_parameter('turn_speed', 0.80)           # rad/s max turning rate
        self.declare_parameter('obstacle_threshold', 1.00)   # m distance where obstacle repulsion begins
        self.declare_parameter('critical_turn_dist', 0.50)   # m distance where sharp turn or reverse is triggered
        self.declare_parameter('mode', 'adaptive')           # 'adaptive' or 'fixed'
        self.declare_parameter('loop_rate_hz', 20.0)

        self.nominal_v = float(self.get_parameter('nominal_speed').value)
        self.max_w = float(self.get_parameter('turn_speed').value)
        self.obs_thresh = float(self.get_parameter('obstacle_threshold').value)
        self.crit_dist = float(self.get_parameter('critical_turn_dist').value)
        self.mode = str(self.get_parameter('mode').value).lower()
        self.rate_hz = float(self.get_parameter('loop_rate_hz').value)

        self.latest_scan: Optional[LaserScan] = None
        self.stuck_counter = 0
        self.reverse_counter = 0
        self.preferred_turn_dir = 1.0  # 1.0 = left, -1.0 = right

        # QoS
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # Subscriber
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, sensor_qos
        )

        # Publishers
        # If adaptive: publish to /cmd_vel_raw (fed into adaptive_speed_controller)
        # If fixed: publish directly to /cmd_vel
        self.cmd_raw_stamped_pub = self.create_publisher(TwistStamped, '/cmd_vel_raw', 10)
        self.cmd_raw_legacy_pub = self.create_publisher(Twist, '/cmd_vel_raw_unstamped', 10)
        self.cmd_direct_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        self.timer = self.create_timer(1.0 / self.rate_hz, self.navigation_loop)

        self.get_logger().info(
            f"Clutter Navigator started in [{self.mode.upper()}] mode | "
            f"Nominal Speed: {self.nominal_v} m/s"
        )

    def scan_callback(self, msg: LaserScan):
        self.latest_scan = msg

    def evaluate_sectors(self):
        """Partitions LiDAR into 5 sectors: Left, Front-Left, Front, Front-Right, Right."""
        if self.latest_scan is None:
            return 5.0, 5.0, 5.0, 5.0, 5.0

        scan = self.latest_scan
        angle = scan.angle_min

        sectors = {
            'right': [],
            'front_right': [],
            'front': [],
            'front_left': [],
            'left': []
        }

        for r in scan.ranges:
            if math.isinf(r) or math.isnan(r) or r < scan.range_min:
                dist = scan.range_max
            else:
                dist = min(r, scan.range_max)

            deg = math.degrees(math.atan2(math.sin(angle), math.cos(angle)))

            if -90.0 <= deg < -40.0:
                sectors['right'].append(dist)
            elif -40.0 <= deg < -15.0:
                sectors['front_right'].append(dist)
            elif -15.0 <= deg <= 15.0:
                sectors['front'].append(dist)
            elif 15.0 < deg <= 40.0:
                sectors['front_left'].append(dist)
            elif 40.0 < deg <= 90.0:
                sectors['left'].append(dist)

            angle += scan.angle_increment

        def safe_min(lst):
            return min(lst) if lst else scan.range_max

        d_r = safe_min(sectors['right'])
        d_fr = safe_min(sectors['front_right'])
        d_f = safe_min(sectors['front'])
        d_fl = safe_min(sectors['front_left'])
        d_l = safe_min(sectors['left'])

        return d_l, d_fl, d_f, d_fr, d_r

    def navigation_loop(self):
        d_l, d_fl, d_f, d_fr, d_r = self.evaluate_sectors()

        # Handle stuck / reverse recovery maneuver
        if self.reverse_counter > 0:
            self.reverse_counter -= 1
            cmd_v = -0.10
            cmd_w = self.preferred_turn_dir * self.max_w * 0.8
            self.publish_cmd(cmd_v, cmd_w)
            return

        # Check if trapped or blocked in front
        if d_f < self.crit_dist and (d_fl < self.crit_dist and d_fr < self.crit_dist):
            self.stuck_counter += 1
            if self.stuck_counter > 15:  # trapped for > 0.75s
                self.reverse_counter = 20  # reverse for 1 second
                self.stuck_counter = 0
                self.preferred_turn_dir = 1.0 if (d_l > d_r) else -1.0
                return
        else:
            self.stuck_counter = 0

        # Reactive Steering Law
        cmd_v = self.nominal_v
        cmd_w = 0.0

        # Evaluate lateral clearance bias
        left_clearance = min(d_l, d_fl)
        right_clearance = min(d_r, d_fr)

        if d_f < self.obs_thresh:
            # Front obstacle detected: initiate turning towards the more open side
            turn_direction = 1.0 if left_clearance >= right_clearance else -1.0
            self.preferred_turn_dir = turn_direction

            # Sharpness of turn inversely proportional to front distance
            urgency = (self.obs_thresh - d_f) / (self.obs_thresh - 0.20 + 1e-5)
            urgency = max(0.0, min(1.0, urgency))

            cmd_w = turn_direction * (0.40 + urgency * (self.max_w - 0.40))
        else:
            # Corridor centering: gentle repulsions from side walls/obstacles
            repulsion = 0.0
            if d_fl < 0.70:
                repulsion -= (0.70 - d_fl) * 1.5
            if d_fr < 0.70:
                repulsion += (0.70 - d_fr) * 1.5
            cmd_w = max(-self.max_w, min(self.max_w, repulsion))

        self.publish_cmd(cmd_v, cmd_w)

    def publish_cmd(self, v: float, w: float):
        now = self.get_clock().now()

        stamped = TwistStamped()
        stamped.header.stamp = now.to_msg()
        stamped.header.frame_id = 'base_footprint'
        stamped.twist.linear.x = v
        stamped.twist.angular.z = w

        legacy = Twist()
        legacy.linear.x = v
        legacy.angular.z = w

        if self.mode == 'adaptive':
            # Feed into adaptive controller which outputs modulated cmd_vel
            self.cmd_raw_stamped_pub.publish(stamped)
            self.cmd_raw_legacy_pub.publish(legacy)
        else:
            # Fixed speed mode: publish directly to robot cmd_vel
            self.cmd_direct_pub.publish(stamped)


def main(args=None):
    rclpy.init(args=args)
    node = ClutterNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
