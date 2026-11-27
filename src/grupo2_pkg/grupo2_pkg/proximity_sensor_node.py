#!/usr/bin/env python3
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Range
import serial


class ProximitySerialNode(Node):
    def __init__(self):
        super().__init__('proximity_serial_node')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('topic', 'proximity')
        self.declare_parameter('frame_id', 'proximity_link')
        self.declare_parameter('min_range', 0.04)
        self.declare_parameter('max_range', 5.0)    

        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value
        self.frame_id = self.get_parameter('frame_id').value
        self.min_r = self.get_parameter('min_range').value
        self.max_r = self.get_parameter('max_range').value

        self.pub = self.create_publisher(
            Range, self.get_parameter('topic').value, 10)

        self.ser = serial.Serial(port, baud, timeout=0)  # no bloqueante
        self.buffer = b''
        self.get_logger().info(f'Escuchando {port} @ {baud}')

        self.create_timer(0.01, self.read_serial)  # 100 Hz

    def read_serial(self):
        try:
            self.buffer += self.ser.read(self.ser.in_waiting or 1)
        except serial.SerialException as e:
            self.get_logger().error(f'Error serial: {e}')
            return

        while b'\n' in self.buffer:
            line, self.buffer = self.buffer.split(b'\n', 1)
            try:
                cm = float(line.decode().strip())
            except ValueError:
                continue  # línea corrupta
            if cm < 0:
                continue  # lectura inválida

            msg = Range()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            msg.radiation_type = Range.ULTRASOUND
            msg.field_of_view = 0.26  # 15 grados aprox
            msg.min_range = self.min_r
            msg.max_range = self.max_r
            msg.range = cm / 100.0    # cm a m
            self.pub.publish(msg)

    def destroy_node(self):
        if self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = ProximitySerialNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()