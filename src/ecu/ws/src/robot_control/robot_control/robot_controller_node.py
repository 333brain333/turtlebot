#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

class RobotSerialController(Node):
    def __init__(self):
        super().__init__('robot_serial_controller')

        # Подписываемся на команду скорости
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10)

        # Настройка Serial порта (замените на свой путь)
        self.serial_port = serial.Serial(
            "/dev/ttyTHS2",
            baudrate=115200,
            timeout=1
        )

    def listener_callback(self, msg):
        linear_velocity = msg.linear.x * 1000  # м/с в мм/с
        angular_velocity = msg.angular.z  # рад/с

        command = f"SET_ROBOT_VELOCITY {linear_velocity:.2f} {angular_velocity:.2f}\n"
        self.serial_port.write(command.encode())
        self.get_logger().info(f"Sent command: {command.strip()}")

def main(args=None):
    rclpy.init(args=args)
    node = RobotSerialController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
