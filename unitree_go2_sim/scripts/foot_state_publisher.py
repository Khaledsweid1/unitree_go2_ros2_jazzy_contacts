#!/usr/bin/env python3
"""Aggregate the four Gazebo foot contact sensors into one /foot_states topic.

Gazebo publishes a ``gz.msgs.Contacts`` message per foot (bridged to ROS as
``ros_gz_interfaces/msg/Contacts``) on
    /foot_contact/{lf,rf,lh,rh}
Each message carries, per contact point, the colliding pair and a world-frame
wrench (``body_1_wrench`` = force on this foot). This node collapses the four
per-foot streams into a single ``champ_msgs/msg/FootStates`` message with a
per-foot contact flag and summed reaction force fx/fy/fz.

Real force values require the dartsim physics engine (bullet-featherstone
returns zeros); contact detection works on either engine.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Vector3
from ros_gz_interfaces.msg import Contacts
from champ_msgs.msg import FootStates


class FootStatePublisher(Node):
    def __init__(self):
        super().__init__("foot_state_publisher")

        self.feet = list(
            self.declare_parameter("feet", ["lf", "rf", "lh", "rh"]).value
        )
        prefix = self.declare_parameter("input_topic_prefix", "/foot_contact/").value
        out_topic = self.declare_parameter("output_topic", "/foot_states").value
        self.rate = float(self.declare_parameter("publish_rate", 100.0).value)
        # A foot whose sensor has not reported within this window is treated as
        # not-in-contact (gz contact sensors go silent when nothing touches).
        self.timeout = float(self.declare_parameter("timeout", 0.05).value)
        # A foot is "in contact" when its measured reaction force magnitude
        # exceeds this (N). Cleanly separates stance feet from swing feet during
        # walking (swing force ~0). 0.1 N per the intended definition.
        self.force_threshold = float(
            self.declare_parameter("contact_force_threshold", 0.1).value
        )

        # Latest state per foot: (in_contact, fx, fy, fz, stamp_seconds).
        self._state = {f: (False, 0.0, 0.0, 0.0, None) for f in self.feet}

        # Match the ros_gz_bridge default publisher QoS (reliable, depth 10).
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._subs = []
        for f in self.feet:
            topic = f"{prefix}{f}"
            self._subs.append(
                self.create_subscription(
                    Contacts, topic, self._make_cb(f), qos
                )
            )
            self.get_logger().info(f"subscribing to {topic}")

        self._pub = self.create_publisher(FootStates, out_topic, 10)
        self.create_timer(1.0 / self.rate, self._publish)
        self.get_logger().info(f"publishing aggregated foot states on {out_topic}")

    def _make_cb(self, foot):
        def cb(msg: Contacts):
            fx = fy = fz = 0.0
            for c in msg.contacts:
                for w in c.wrenches:
                    fx += w.body_1_wrench.force.x
                    fy += w.body_1_wrench.force.y
                    fz += w.body_1_wrench.force.z
            # in contact iff the reaction force magnitude exceeds the threshold
            fmag = (fx * fx + fy * fy + fz * fz) ** 0.5
            touching = fmag > self.force_threshold
            now = self.get_clock().now().nanoseconds * 1e-9
            self._state[foot] = (touching, fx, fy, fz, now)

        return cb

    def _publish(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        out = FootStates()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "odom"  # forces are expressed in the world frame
        for f in self.feet:
            in_contact, fx, fy, fz, stamp = self._state[f]
            if stamp is None or (now - stamp) > self.timeout:
                in_contact, fx, fy, fz = False, 0.0, 0.0, 0.0
            out.names.append(f)
            out.in_contact.append(bool(in_contact))
            out.force.append(Vector3(x=fx, y=fy, z=fz))
            out.normal_force.append(abs(fz))
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = FootStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
