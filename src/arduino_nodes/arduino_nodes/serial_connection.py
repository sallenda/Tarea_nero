#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial
import time

class ArduinoSerialNode(Node):
    def __init__(self):
        super().__init__('arduino_serial_node')
        
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        
        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value

        try:
            self.ser = serial.Serial(port, baudrate, timeout=0.05)
            time.sleep(2.0)
            self.ser.reset_input_buffer()
            self.get_logger().info(f'Conectado a Arduino en {port} a {baudrate} baudios.')
        except serial.SerialException as e:
            self.get_logger().error(f'Error abriendo el puerto serial: {e}')
            raise e

        self.publisher_ = self.create_publisher(Float32, 'ultrasonic/distance_cm', 10)

        self.timer = self.create_timer(0.02, self.read_serial_callback)

    def read_serial_callback(self):
        if self.ser.in_waiting > 0:
            try:
                line = self.ser.readline().decode('utf-8').strip()
                if line:
                    distance = float(line)  #Conversion a flotante
                    if distance >= 0.0:     #Descarta el -1.0 (lectura invalida)
                        msg = Float32()
                        msg.data = distance
                        self.publisher_.publish(msg)
            except (UnicodeDecodeError, ValueError):
                # Ignora bytes corruptos o tramas cortadas al abrir el puerto
                pass

    def destroy_node(self):
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoSerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
