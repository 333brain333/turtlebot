#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose2D, Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String

import tf2_ros
import serial
import re
from transforms3d.euler import euler2quat


class RobotDriverNode(Node):
    def __init__(self):
        super().__init__("robot_driver_node")

        # 1. Подписка на /cmd_vel для приёма скоростей
        self.cmd_vel_sub = self.create_subscription(
            Twist, "/cmd_vel", self.cmd_vel_callback, 10
        )
        self.set_pose_sub = self.create_subscription(
            Pose2D, "/set_pose", self.set_pose_callback, 10
        )
        self.set_led_sub = self.create_subscription(
            String, "/SET_LED", self.set_led_callback, 10
        )
        self.shutdown_sub = self.create_subscription(
            String, "/SHUTDOWN", self.shutdown_callback, 10
        )
        self.set_coeff_sub = self.create_subscription(
            String, "/SET_COEFF", self.set_coeff_callback, 10
        )

        # 2. Паблишер одометрии
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.raw_esp32_pub = self.create_publisher(String, "/ESP32_RAW", 50)

        # 3. TF Broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # 4. Открываем Serial-порт
        serial_port_path = "/dev/ttyTHS2"
        self.ser = serial.Serial(serial_port_path, baudrate=115200, timeout=0.1)
        self.get_logger().info(f"Opened serial port: {serial_port_path}")

        # 5. Регулярка для парсинга строк вида: "POS X=120.00 Y=150.00 Th=0.45"
        self.pos_pattern = re.compile(r"POS X=([\-\d\.]+) Y=([\-\d\.]+) Th=([\-\d\.]+)")

        # 6. Периодический таймер для чтения данных из Serial
        self.timer_period = 0.05  # 20 Гц
        self.read_serial_timer = self.create_timer(
            self.timer_period, self.read_serial_timer_callback
        )

    def write_serial_command(self, command: str):
        command = command.strip()
        if not command:
            return
        try:
            self.ser.write((command + "\n").encode("utf-8"))
            self.get_logger().info(f"Sent command: {command}")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to write to serial: {e}")

    def set_pose_callback(self, msg: Pose2D):
        x = msg.x * 1000
        y = msg.y * 1000
        command = f"SET_POSE {x:.2f} {y:.2f} {msg.theta:.2f}\n"
        self.write_serial_command(command)

    def cmd_vel_callback(self, msg: Twist):
        """
        Колбэк, вызывается при приходе сообщения в /cmd_vel
        Отправляем команду скорости на ESP32 по Serial.
        """
        linear_velocity = msg.linear.x * 1000.0  # м/с -> мм/с
        angular_velocity = msg.angular.z  # рад/с

        command = f"SET_ROBOT_VELOCITY {linear_velocity:.2f} {angular_velocity:.2f}\n"
        self.write_serial_command(command)

    def set_led_callback(self, msg: String):
        self.write_serial_command(msg.data)

    def shutdown_callback(self, msg: String):
        self.write_serial_command(msg.data or "SHUTDOWN")

    def set_coeff_callback(self, msg: String):
        self.write_serial_command(msg.data)

    def read_serial_timer_callback(self):
        """
        Периодически читаем строки из Serial.
        Если строка похожа на POS X=... Y=... Th=..., парсим и публикуем одометрию + TF.
        """
        # Считываем всё, что накопилось в буфере (т.к. timeout=0.1)
        try:
            # Можем читать построчно, т.к. ESP32 шлёт строки.
            line = self.ser.readline().decode("ascii", errors="ignore").strip()

            if not line:
                return  # нет данных в данный момент

            self.raw_esp32_pub.publish(String(data=line))

            # Проверим, похоже ли это на строку одометрии
            match = self.pos_pattern.search(line)
            if match:
                x_mm = float(match.group(1))  # мм
                y_mm = float(match.group(2))  # мм
                th = float(match.group(3))  # рад

                # Переводим мм -> м
                x = x_mm / 1000.0
                y = y_mm / 1000.0

                # Формируем Odometry
                odom_msg = Odometry()
                odom_msg.header.stamp = self.get_clock().now().to_msg()
                odom_msg.header.frame_id = "odom"
                odom_msg.child_frame_id = "base_link"

                odom_msg.pose.pose.position.x = x
                odom_msg.pose.pose.position.y = y
                odom_msg.pose.pose.position.z = 0.0

                # Преобразуем угол в кватернион
                q = euler2quat(
                    0.0, 0.0, th
                )  # returns (w, x, y, z) - такая у transforms3d специфика
                odom_msg.pose.pose.orientation.w = q[0]
                odom_msg.pose.pose.orientation.x = q[1]
                odom_msg.pose.pose.orientation.y = q[2]
                odom_msg.pose.pose.orientation.z = q[3]

                # Публикуем в топик /odom
                self.odom_pub.publish(odom_msg)

                # Одновременно шлём TF odom -> base_link
                t = TransformStamped()
                t.header.stamp = odom_msg.header.stamp
                t.header.frame_id = "odom"
                t.child_frame_id = "base_link"
                t.transform.translation.x = x
                t.transform.translation.y = y
                t.transform.translation.z = 0.0
                t.transform.rotation.w = q[0]
                t.transform.rotation.x = q[1]
                t.transform.rotation.y = q[2]
                t.transform.rotation.z = q[3]

                self.tf_broadcaster.sendTransform(t)

                # Для наглядности выведем в лог
                self.get_logger().info(f"ODOM: X={x:.3f} Y={y:.3f} Th={th:.3f}")
            else:
                # Можно выводить предупреждение, но чтобы не заспамить лог, сделаем debug
                self.get_logger().debug(f"Unknown line: {line}")

        except serial.SerialException as e:
            self.get_logger().error(f"Failed to read from serial: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = RobotDriverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
