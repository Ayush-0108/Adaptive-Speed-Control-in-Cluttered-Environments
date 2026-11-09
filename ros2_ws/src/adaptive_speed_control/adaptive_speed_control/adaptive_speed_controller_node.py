#!/usr/bin/env python3
"""
Adaptive Speed Controller Node for ROS 2.
Dynamically regulates linear and angular velocity based on LiDAR obstacle proximity,
environmental clutter density, and curvature constraints.
"""

import json
import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist, TwistStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Header
from visualization_msgs.msg import Marker, MarkerArray


class AdaptiveSpeedController(Node):
    def __init__(self):
        super().__init__('adaptive_speed_controller')

        # Declare configurable parameters
        self.declare_parameter('max_linear_velocity', 0.50)       # m/s in open space
        self.declare_parameter('min_linear_velocity', 0.08)       # m/s minimum crawl speed
        self.declare_parameter('critical_distance', 0.35)         # m (emergency brake radius)
        self.declare_parameter('safe_distance', 1.50)             # m (nominal full-speed distance)
        self.declare_parameter('clutter_eval_radius', 2.00)       # m (radius to compute density)
        self.declare_parameter('clutter_penalty_weight', 0.65)    # weight for clutter deceleration
        self.declare_parameter('min_clutter_factor', 0.25)        # lowest speed factor in max clutter
        self.declare_parameter('angular_speed_penalty', 1.20)     # turn curvature deceleration factor
        self.declare_parameter('max_linear_accel', 0.80)          # m/s^2 forward acceleration
        self.declare_parameter('max_linear_decel', 2.20)          # m/s^2 forward braking deceleration
        self.declare_parameter('control_rate_hz', 50.0)           # Hz update loop frequency
        self.declare_parameter('front_wedge_angle_deg', 35.0)     # degrees (+/-) for forward collision cone

        # Read parameters
        self.max_v = float(self.get_parameter('max_linear_velocity').value)
        self.min_v = float(self.get_parameter('min_linear_velocity').value)
        self.d_critical = float(self.get_parameter('critical_distance').value)
        self.d_safe = float(self.get_parameter('safe_distance').value)
        self.r_clutter = float(self.get_parameter('clutter_eval_radius').value)
        self.k_clutter = float(self.get_parameter('clutter_penalty_weight').value)
        self.min_clutter_factor = float(self.get_parameter('min_clutter_factor').value)
        self.k_turn = float(self.get_parameter('angular_speed_penalty').value)
        self.a_accel = float(self.get_parameter('max_linear_accel').value)
        self.a_decel = float(self.get_parameter('max_linear_decel').value)
        self.control_rate = float(self.get_parameter('control_rate_hz').value)
        self.front_cone_rad = math.radians(float(self.get_parameter('front_wedge_angle_deg').value))

        # Sensor & Command state
        self.latest_scan: Optional[LaserScan] = None
        self.raw_cmd_v: float = 0.0
        self.raw_cmd_w: float = 0.0
        self.last_cmd_time = self.get_clock().now()
        self.current_output_v: float = 0.0
        self.current_output_w: float = 0.0

        # Analysis metrics
        self.min_front_distance: float = 10.0
        self.clutter_index: float = 0.0
        self.speed_factor: float = 1.0
        self.safety_state: str = "OPEN"

        # QoS configuration
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, sensor_qos
        )
        self.cmd_raw_stamped_sub = self.create_subscription(
            TwistStamped, '/cmd_vel_raw', self.cmd_raw_stamped_callback, 10
        )
        self.cmd_raw_unstamped_sub = self.create_subscription(
            Twist, '/cmd_vel_raw_unstamped', self.cmd_raw_unstamped_callback, 10
        )

        # Publishers
        self.cmd_vel_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.cmd_vel_legacy_pub = self.create_publisher(Twist, '/cmd_vel_legacy', 10)
        self.metrics_pub = self.create_publisher(String, '/speed_controller/metrics', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/speed_controller/markers', 10)

        # Control loop timer (50 Hz)
        self.timer = self.create_timer(1.0 / self.control_rate, self.control_loop)

        self.get_logger().info(
            f"Adaptive Speed Controller initialized | v_max={self.max_v:.2f} m/s | "
            f"d_critical={self.d_critical:.2f} m | d_safe={self.d_safe:.2f} m"
        )

    def scan_callback(self, msg: LaserScan):
        self.latest_scan = msg

    def cmd_raw_stamped_callback(self, msg: TwistStamped):
        self.raw_cmd_v = msg.twist.linear.x
        self.raw_cmd_w = msg.twist.angular.z
        self.last_cmd_time = self.get_clock().now()

    def cmd_raw_unstamped_callback(self, msg: Twist):
        self.raw_cmd_v = msg.linear.x
        self.raw_cmd_w = msg.angular.z
        self.last_cmd_time = self.get_clock().now()

    def process_lidar(self):
        """Analyze LiDAR scan to compute proximity and clutter metrics."""
        if self.latest_scan is None:
            self.min_front_distance = self.d_safe
            self.clutter_index = 0.0
            return

        scan = self.latest_scan
        angle = scan.angle_min
        front_distances = []
        clutter_scores = []

        for r in scan.ranges:
            # Filter non-finite or invalid readings
            if math.isinf(r) or math.isnan(r) or r < scan.range_min:
                dist = scan.range_max
            else:
                dist = min(r, scan.range_max)

            # Normalize angle to [-pi, +pi]
            norm_angle = math.atan2(math.sin(angle), math.cos(angle))

            # 1. Forward Safety Wedge Evaluation
            if abs(norm_angle) <= self.front_cone_rad:
                front_distances.append(dist)

            # 2. Forward Hemisphere Clutter Evaluation ([-pi/2, +pi/2])
            if abs(norm_angle) <= math.pi / 2.0:
                if dist < self.r_clutter:
                    # Score inversely proportional to distance (closer = higher clutter weight)
                    clutter_scores.append(1.0 - (dist / self.r_clutter))
                else:
                    clutter_scores.append(0.0)

            angle += scan.angle_increment

        # Minimum distance in front wedge
        if front_distances:
            self.min_front_distance = min(front_distances)
        else:
            self.min_front_distance = scan.range_max

        # Average clutter index across forward hemisphere (bounded [0.0, 1.0])
        if clutter_scores:
            self.clutter_index = sum(clutter_scores) / len(clutter_scores)
        else:
            self.clutter_index = 0.0

    def compute_adaptive_speed(self) -> float:
        """
        Calculates safe modulated speed factor based on proximity, clutter, and heading.
        """
        # 1. Proximity Scaling Factor S_prox
        if self.min_front_distance <= self.d_critical:
            s_prox = 0.0
            self.safety_state = "EMERGENCY_STOP"
        elif self.min_front_distance < self.d_safe:
            # Smooth non-linear braking scaling
            normalized_d = (self.min_front_distance - self.d_critical) / (self.d_safe - self.d_critical)
            s_prox = math.pow(max(0.0, min(1.0, normalized_d)), 1.2)
            self.safety_state = "BRAKING" if s_prox < 0.3 else "APPROACHING"
        else:
            s_prox = 1.0
            self.safety_state = "OPEN"

        # 2. Environmental Clutter Scaling Factor S_clutter
        # In highly cluttered fields, reduce peak velocity to ensure tighter reaction envelope
        s_clutter = max(self.min_clutter_factor, 1.0 - (self.k_clutter * self.clutter_index))
        if self.clutter_index > 0.4 and self.safety_state != "EMERGENCY_STOP":
            self.safety_state = "DENSE_CLUTTER"

        # 3. Angular Turn Scaling Factor S_turn
        s_turn = 1.0 / (1.0 + (self.k_turn * abs(self.raw_cmd_w)))

        # Composite speed factor
        total_factor = s_prox * s_clutter * s_turn
        self.speed_factor = max(0.0, min(1.0, total_factor))

        # Modulate desired linear velocity
        if self.raw_cmd_v > 0.0:
            target_v = self.raw_cmd_v * self.speed_factor
            # If not in emergency stop and moving forward, enforce minimum crawl speed for controllability
            if target_v > 0.0 and self.safety_state != "EMERGENCY_STOP":
                target_v = max(self.min_v, target_v)
            target_v = min(self.max_v, target_v)
        elif self.raw_cmd_v < 0.0:
            # Reverse driving: scale symmetrically
            target_v = self.raw_cmd_v * 0.7
        else:
            target_v = 0.0

        return target_v

    def apply_dynamic_smoothing(self, target_v: float, target_w: float, dt: float):
        """
        Applies acceleration and deceleration limits to prevent dynamic instability and wheel slip.
        """
        # Deceleration (braking) allows higher rate than acceleration
        if abs(target_v) < abs(self.current_output_v):
            max_delta = self.a_decel * dt
        else:
            max_delta = self.a_accel * dt

        diff_v = target_v - self.current_output_v
        if abs(diff_v) <= max_delta:
            self.current_output_v = target_v
        else:
            self.current_output_v += math.copysign(max_delta, diff_v)

        # Angular velocity follows target directly with slight damping
        self.current_output_w = target_w

    def control_loop(self):
        now = self.get_clock().now()
        dt = 1.0 / self.control_rate

        # Command timeout safety: stop if no commands received in last 0.5s
        cmd_age = (now - self.last_cmd_time).nanoseconds / 1e9
        if cmd_age > 0.6:
            self.raw_cmd_v = 0.0
            self.raw_cmd_w = 0.0

        # Process sensor readings
        self.process_lidar()

        # Compute adaptive speed
        target_v = self.compute_adaptive_speed()
        target_w = self.raw_cmd_w

        # Smooth velocities
        self.apply_dynamic_smoothing(target_v, target_w, dt)

        # Publish TwistStamped for Gazebo Harmonic DiffDrive
        twist_stamped = TwistStamped()
        twist_stamped.header.stamp = now.to_msg()
        twist_stamped.header.frame_id = 'base_footprint'
        twist_stamped.twist.linear.x = self.current_output_v
        twist_stamped.twist.linear.y = 0.0
        twist_stamped.twist.linear.z = 0.0
        twist_stamped.twist.angular.x = 0.0
        twist_stamped.twist.angular.y = 0.0
        twist_stamped.twist.angular.z = self.current_output_w
        self.cmd_vel_pub.publish(twist_stamped)

        # Also publish legacy Twist
        legacy_twist = Twist()
        legacy_twist.linear.x = self.current_output_v
        legacy_twist.angular.z = self.current_output_w
        self.cmd_vel_legacy_pub.publish(legacy_twist)

        # Publish telemetry and RViz markers
        self.publish_metrics()
        self.publish_rviz_markers()

    def publish_metrics(self):
        data = {
            "current_speed_mps": round(self.current_output_v, 3),
            "raw_requested_mps": round(self.raw_cmd_v, 3),
            "angular_velocity_radps": round(self.current_output_w, 3),
            "min_front_distance_m": round(self.min_front_distance, 3),
            "clutter_density_percent": round(self.clutter_index * 100.0, 1),
            "speed_factor_percent": round(self.speed_factor * 100.0, 1),
            "safety_state": self.safety_state
        }
        msg = String()
        msg.data = json.dumps(data)
        self.metrics_pub.publish(msg)

    def publish_rviz_markers(self):
        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()

        # 1. Proximity Safety Zone Bubble
        bubble = Marker()
        bubble.header.frame_id = "base_footprint"
        bubble.header.stamp = now
        bubble.ns = "safety_zone"
        bubble.id = 0
        bubble.type = Marker.CYLINDER
        bubble.action = Marker.ADD
        bubble.pose.position.x = max(0.0, self.min_front_distance / 2.0)
        bubble.pose.position.y = 0.0
        bubble.pose.position.z = 0.1
        bubble.scale.x = max(0.4, float(self.min_front_distance))
        bubble.scale.y = 0.6
        bubble.scale.z = 0.05

        # Dynamic color coding based on safety state
        if self.safety_state == "EMERGENCY_STOP":
            bubble.color.r, bubble.color.g, bubble.color.b, bubble.color.a = 1.0, 0.0, 0.0, 0.6
        elif self.safety_state in ["BRAKING", "DENSE_CLUTTER"]:
            bubble.color.r, bubble.color.g, bubble.color.b, bubble.color.a = 1.0, 0.5, 0.0, 0.4
        elif self.safety_state == "APPROACHING":
            bubble.color.r, bubble.color.g, bubble.color.b, bubble.color.a = 1.0, 0.9, 0.0, 0.4
        else:
            bubble.color.r, bubble.color.g, bubble.color.b, bubble.color.a = 0.0, 1.0, 0.2, 0.3

        marker_array.markers.append(bubble)

        # 2. HUD Text Marker floating above robot
        text_marker = Marker()
        text_marker.header.frame_id = "base_footprint"
        text_marker.header.stamp = now
        text_marker.ns = "speed_hud"
        text_marker.id = 1
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.pose.position.x = 0.0
        text_marker.pose.position.y = 0.0
        text_marker.pose.position.z = 0.65
        text_marker.scale.z = 0.14
        text_marker.color.r, text_marker.color.g, text_marker.color.b, text_marker.color.a = 1.0, 1.0, 1.0, 1.0
        text_marker.text = (
            f"Speed: {self.current_output_v:.2f} m/s ({int(self.speed_factor*100)}%)\n"
            f"Clutter: {int(self.clutter_index*100)}% | Dist: {self.min_front_distance:.2f}m\n"
            f"State: {self.safety_state}"
        )
        marker_array.markers.append(text_marker)

        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = AdaptiveSpeedController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
