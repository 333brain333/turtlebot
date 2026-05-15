import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import serial
import tf2_ros
from transforms3d.euler import euler2quat
import re

class SerialOdomNode(Node):
    def __init__(self):
        super().__init__('serial_to_odom')

        # Serial-порт (настрой свой путь)
        self.ser = serial.Serial(
            "/dev/ttyTHS2", 115200, timeout=1)
        self.ser.write(b"PING\n")
        # Паблишер одометрии
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)

        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Регулярное выражение для парсинга строки
        self.pattern = re.compile(r'POS X=([\-\d\.]+) Y=([\-\d\.]+) Th=([\-\d\.]+)')

        # Таймер чтения данных из Serial
        self.timer = self.create_timer(0.05, self.timer_callback)

    def timer_callback(self):
        if self.ser.in_waiting > 0:
            line = self.ser.readline().decode('ascii', errors='ignore').strip()
            match = self.pattern.search(line)

            if match:
                x = float(match.group(1)) / 1000.0  # мм → метры
                y = float(match.group(2)) / 1000.0
                th = float(match.group(3))

                # Создаем сообщение Odometry
                odom = Odometry()
                odom.header.stamp = self.get_clock().now().to_msg()
                odom.header.frame_id = 'odom'
                odom.child_frame_id = 'base_link'
                odom.pose.pose.position.x = x
                odom.pose.pose.position.y = y

                quat = euler2quat(0, 0, th)
                odom.pose.pose.orientation.w = quat[0]
                odom.pose.pose.orientation.x = quat[1]
                odom.pose.pose.orientation.y = quat[2]
                odom.pose.pose.orientation.z = quat[3]

                self.odom_pub.publish(odom)

                # Отправляем TF-трансформацию odom → base_link
                t = TransformStamped()
                t.header.stamp = odom.header.stamp
                t.header.frame_id = 'odom'
                t.child_frame_id = 'base_link'
                t.transform.translation.x = x
                t.transform.translation.y = y
                t.transform.translation.z = 0.0
                t.transform.rotation.w = quat[0]
                t.transform.rotation.x = quat[1]
                t.transform.rotation.y = quat[2]
                t.transform.rotation.z = quat[3]

                self.tf_broadcaster.sendTransform(t)

                self.get_logger().info(f"X: {x:.3f}, Y: {y:.3f}, Th: {th:.3f}")

            else:
                self.get_logger().warn(f"Не удалось разобрать строку: {line}")

def main(args=None):
    rclpy.init(args=args)
    node = SerialOdomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
