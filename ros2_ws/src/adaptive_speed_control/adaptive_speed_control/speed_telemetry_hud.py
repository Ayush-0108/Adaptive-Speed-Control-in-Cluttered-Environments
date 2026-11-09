#!/usr/bin/env python3
"""
Speed Telemetry HUD Node.
Renders a real-time ASCII dashboard showing speed modulation, clutter index,
and safety state directly in the terminal console.
"""

import json
import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SpeedTelemetryHUD(Node):
    def __init__(self):
        super().__init__('speed_telemetry_hud')

        self.sub = self.create_subscription(
            String,
            '/speed_controller/metrics',
            self.metrics_callback,
            10
        )
        self.last_render_time = self.get_clock().now()

        print("\n" + "=" * 65)
        print("  ADAPTIVE SPEED CONTROL TELEMETRY HUD  (ROS 2 Jazzy)")
        print("=" * 65)
        print("Listening on /speed_controller/metrics...\n")

    def make_bar(self, value: float, max_val: float, length: int = 20, fill_char: str = "█") -> str:
        ratio = max(0.0, min(1.0, value / max_val if max_val > 0 else 0.0))
        filled = int(ratio * length)
        return fill_char * filled + "░" * (length - filled)

    def metrics_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
        except Exception:
            return

        now = self.get_clock().now()
        # Rate limit console refresh to 4 Hz for readability
        if (now - self.last_render_time).nanoseconds / 1e9 < 0.25:
            return
        self.last_render_time = now

        v = data.get("current_speed_mps", 0.0)
        v_raw = data.get("raw_requested_mps", 0.0)
        w = data.get("angular_velocity_radps", 0.0)
        dist = data.get("min_front_distance_m", 0.0)
        clutter = data.get("clutter_density_percent", 0.0)
        factor = data.get("speed_factor_percent", 100.0)
        state = data.get("safety_state", "UNKNOWN")

        speed_bar = self.make_bar(v, 0.50, length=20)
        clutter_bar = self.make_bar(clutter, 100.0, length=20)

        # State badge
        state_badge = f"[{state:^16}]"

        sys.stdout.write("\033[F" * 6 if hasattr(self, '_printed') else "")
        self._printed = True

        out = (
            f"┌─────────────────────────────────────────────────────────────┐\n"
            f"│ Velocity:  {v:5.2f} m/s |{speed_bar}| Req:{v_raw:5.2f} m/s │\n"
            f"│ Clutter:   {clutter:5.1f} %   |{clutter_bar}| Scale:{factor:3.0f}%     │\n"
            f"│ Front Dist:{dist:5.2f} m   │ Turn: {w:+5.2f} rad/s │ State:{state_badge}│\n"
            f"└─────────────────────────────────────────────────────────────┘\n"
        )
        sys.stdout.write(out)
        sys.stdout.flush()


def main(args=None):
    rclpy.init(args=args)
    node = SpeedTelemetryHUD()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
