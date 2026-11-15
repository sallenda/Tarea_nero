#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class DepthSensorNode(Node):
    def __init__(self):
        super().__init__('depth_sensor_node')
        self.declare_parameter('input_topic', 'ultrasonic/distance_cm')
        self.declare_parameter('output_topic', 'depth_sensor/distance_cm')

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        self.distance_pub = self.create_publisher(Float32, output_topic, 10)
        self.create_subscription(Float32, input_topic, self._on_distance, 10)
        self.get_logger().info(
            f'Sensor de profundidad conectado: {input_topic} -> {output_topic} (cm).'
        )

    def _on_distance(self, msg: Float32):
        self.distance_pub.publish(msg)
        self.get_logger().info(f'Distancia ultrasonica: {msg.data:.2f} cm')


def main(args=None):
    rclpy.init(args=args)
    node = DepthSensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

        

# cd ~/Projects/Tarea_nero
# source /opt/ros/jazzy/setup.bash
# source install/setup.bash

# ros2 launch nero_ctrl nero_sim_ik_depth.launch.py \
#   demo:=true \
#   serial_port:=/dev/ttyACM0

# En otra terminal
# source /opt/ros/jazzy/setup.bash
# source ~/Projects/Tarea_nero/install/setup.bash

# ros2 topic echo /depth_sensor/distance_cm
