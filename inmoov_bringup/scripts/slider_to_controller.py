#!/usr/bin/env python3
"""Let the RViz sliders drive a ros2_control controller.

The joint_state_publisher_gui publishes a JointState on /joint_states. In the controller
path that topic belongs to joint_state_broadcaster — it is the robot telling you where it
IS, not you telling it where to GO — so the sliders are remapped onto slider_joint_states
and this node forwards them to the controller's command topic instead.

Without this, "sliders" and "controller" are two mutually exclusive launch files, which is
exactly the confusion this is meant to remove.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class SliderToController(Node):
    def __init__(self):
        super().__init__("slider_to_controller")
        self.declare_parameter("joints", [""])
        self.declare_parameter(
            "command_topic", "/inmoov_position_controller/commands")

        self.joints = [j for j in self.get_parameter("joints").value if j]
        topic = self.get_parameter("command_topic").value
        if not self.joints:
            self.get_logger().error(
                "no 'joints' parameter — nothing to forward. The launch file normally "
                "reads this from inmoov_controllers.yaml so the ORDER matches the "
                "controller's own joint list; order matters, names are not sent.")

        self.latest = {}
        self.warned = False
        self.pub = self.create_publisher(Float64MultiArray, topic, 10)
        self.sub = self.create_subscription(
            JointState, "slider_joint_states", self.on_sliders, 10)
        self.get_logger().info(
            f"forwarding {len(self.joints)} slider joint(s) -> {topic}")

    def on_sliders(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            self.latest[name] = pos

        missing = [j for j in self.joints if j not in self.latest]
        if missing:
            # The GUI publishes every joint at once, so this only trips on a genuine
            # name mismatch between the controller list and the URDF.
            if not self.warned:
                self.warned = True
                self.get_logger().warn(
                    f"waiting for {len(missing)} joint(s) not present in the sliders, "
                    f"e.g. {missing[:3]}")
            return

        out = Float64MultiArray()
        out.data = [float(self.latest[j]) for j in self.joints]
        self.pub.publish(out)


def main():
    rclpy.init()
    node = SliderToController()
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
