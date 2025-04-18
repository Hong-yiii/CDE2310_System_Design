#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import board
import busio
import adafruit_amg88xx
from std_msgs.msg import Float32MultiArray, MultiArrayLayout, MultiArrayDimension

class AMG8833DualPublisher(Node):
    def __init__(self):
        super().__init__('amg8833_dual_publisher')

        # Create publishers for both sensors
        self.publisher_1 = self.create_publisher(Float32MultiArray, 'temperature_sensor_1', 10)
        self.publisher_2 = self.create_publisher(Float32MultiArray, 'temperature_sensor_2', 10)

        # Set up publishing rate (1 Hz)
        self.timer = self.create_timer(1.0, self.publish_data)

        # Initialize I2C bus
        i2c_bus = busio.I2C(board.SCL, board.SDA)

        # Explicitly set I2C addresses for both sensors
        self.sensor_1 = adafruit_amg88xx.AMG88XX(i2c_bus, addr=0x68)  # First sensor (0x68)
        self.sensor_2 = adafruit_amg88xx.AMG88XX(i2c_bus, addr=0x69)  # Second sensor (0x69)

    def publish_data(self):
        """ Reads and publishes data from both AMG8833 sensors. """

        # Read data from both sensors
        matrix_1 = np.array(self.sensor_1.pixels, dtype=np.float32)
        matrix_2 = np.array(self.sensor_2.pixels, dtype=np.float32)

        # Convert to ROS 2 messages
        temp_data_1 = self.create_temperature_message(matrix_1)
        temp_data_2 = self.create_temperature_message(matrix_2)

        # Publish data to respective topics
        self.publisher_1.publish(temp_data_1)
        self.publisher_2.publish(temp_data_2)

        # Log output
        self.get_logger().info(f"Published AMG8833 Sensor 1 data:\n{matrix_1}")
        self.get_logger().info(f"Published AMG8833 Sensor 2 data:\n{matrix_2}")

    def create_temperature_message(self, matrix):
        """ Helper function to format the temperature data into a Float32MultiArray message. """
        temp_data = Float32MultiArray()
        temp_data.data = matrix.flatten().tolist()

        # Define the MultiArray layout (8x8)
        temp_data.layout = MultiArrayLayout()
        temp_data.layout.dim.append(MultiArrayDimension())
        temp_data.layout.dim.append(MultiArrayDimension())

        temp_data.layout.dim[0].label = "rows"
        temp_data.layout.dim[0].size = 8
        temp_data.layout.dim[0].stride = 64  # 8x8 matrix

        temp_data.layout.dim[1].label = "cols"
        temp_data.layout.dim[1].size = 8
        temp_data.layout.dim[1].stride = 8  # Each row has 8 elements

        return temp_data

def main(args=None):
    rclpy.init(args=args)
    node = AMG8833DualPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
