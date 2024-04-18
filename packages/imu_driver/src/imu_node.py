#!/usr/bin/env python3

import math
from typing import Optional

import adafruit_mpu6050
import board
import rclpy
import yaml
from adafruit_mpu6050 import MPU6050
from sensor_msgs.msg import Imu, Temperature
from std_srvs.srv import Empty

from hardware_test_imu import HardwareTestIMU
from duckietown.dtros import DTROS, NodeType

# TODO: calibration and loading custom config

class IMUNode(DTROS):
    def __init__(self):
        # Node Init
        super(IMUNode, self).__init__(node_name="imu_node", node_type=NodeType.DRIVER)

        # get ROS/Duckiebot parameters
        self._veh = self.get_parameter('veh').get_parameter_value().string_value
        self._i2c_connectors = self.get_parameter("connectors").get_parameter_value().string_value
        self._polling_hz = self.get_parameter("polling_hz").get_parameter_value().double_value
        self._temp_offset = self.get_parameter("temp_offset").get_parameter_value().double_value
        self._gyro_offset = self.get_parameter("ang_vel_offset").get_parameter_value().double_value
        self._accel_offset = self.get_parameter("accel_offset").get_parameter_value().double_value
        self.get_logger().info("===============IMU Node Init Val===============")
        self.get_logger().info(f"Op Rate: {self._polling_hz}")
        self.get_logger().info("Acceleration Offset: X:%.2f, Y: %.2f, Z: %.2f m/s^2" % tuple(self._accel_offset))
        self.get_logger().info("Gyro Offset X:%.2f, Y: %.2f, Z: %.2f degrees/s" % tuple(self._gyro_offset))
        self.get_logger().info("Temp Offset: %.2f C" % self._temp_offset)
        self.get_logger().info("===============END of IMU Init Val===============")
        # IMU Initialization
        self._sensor: Optional[MPU6050] = self._find_sensor()
        if not self._sensor:
            conns: str = yaml.safe_dump(self._i2c_connectors, indent=2, sort_keys=True)
            self.get_logger().error(f"No MPU6050 device found. These connectors were tested:\n{conns}\n")
            exit(1)
        # ---
        self.get_logger().info("===============Performing Initial Testing!===============")
        self.get_logger().info("Acceleration: X:%.2f, Y: %.2f, Z: %.2f m/s^2" % self._sensor.acceleration)
        self.get_logger().info("Gyro X:%.2f, Y: %.2f, Z: %.2f degrees/s" % self._sensor.gyro)
        self.get_logger().info("Temperature: %.2f C" % self._sensor.temperature)
        self.get_logger().info("===============IMU Initialization Complete===============")
        # ROS Pubsub initialization
        self.pub = self.create_publisher(Imu, 'data', 10)
        self.temp_pub = self.create_publisher(Temperature, 'temperature', 10)
        self.create_service(Empty, "initialize_imu", self.zero_sensor)
        self.timer = self.create_timer(1.0 / self._polling_hz, self.publish_data)

        # user hardware test
        self._hardware_test = HardwareTestIMU()

    def _find_sensor(self) -> Optional[MPU6050]:
        for connector in self._i2c_connectors:
            conn: str = "[bus:{bus}](0x{address:02X})".format(**connector)
            self.get_logger().info(f"Trying to open device on connector {conn}")
            # Overwrite Adafruit default device ID
            adafruit_mpu6050._MPU6050_DEVICE_ID = connector["address"]
            try:
                sensor = MPU6050(board.I2C())
            except Exception:
                self.get_logger().warn(f"No devices found on connector {conn}, but the bus exists")
                continue
            self.get_logger().info(f"Device found on connector {conn}")
            return sensor

    def publish_data(self):
        # Message Blank
        msg = Imu()
        temp_msg = Temperature()
        # Poll Sensor
        try:
            # You take the time immediately when you are polling imu
            msg.header.stamp = temp_msg.header.stamp = self.get_clock().now().to_msg()
            acc_data = self._sensor.acceleration
            gyro_data = self._sensor.gyro
            temp_data = self._sensor.temperature
            # Do it together so that the timestamp is honored
            # Populate Message
            msg.header.frame_id = temp_msg.header.frame_id = f"{self._veh}/imu"
            # Orientation (we do not have this data)
            msg.orientation.x = msg.orientation.y = msg.orientation.z = msg.orientation.w = 0
            # If you have no estimate for one of the data elements
            # set element 0 of the associated covariance matrix to -1
            msg.orientation_covariance = [0.0 for _ in range(len(msg.orientation_covariance))]
            msg.orientation_covariance[0] = -1
            # Angular Velocity
            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = tuple(
                gyro_data[i] - self._gyro_offset[i] for i in range(len(gyro_data))
            )
            msg.angular_velocity_covariance = [0.0 for _ in range(len(msg.angular_velocity_covariance))]
            # Acceleration
            msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = tuple(
                (acc_data[i] - self._accel_offset[i]) for i in range(len(acc_data)))
            msg.linear_acceleration_covariance = [0.0 for _ in range(len(msg.linear_acceleration_covariance))]
            # Pub
            self.pub.publish(msg)
            temp_msg.temperature = temp_data - self._temp_offset
            self.temp_pub.publish(temp_msg)

        except Exception as IMUCommLoss:
            self.get_logger().warn(f"IMU Comm Loss: {IMUCommLoss}")
            pass
        return

    def zero_sensor(self, _):
        acc_data = self._sensor.acceleration
        gyro_data = self._sensor.gyro
        temp_data = self._sensor.temperature
        self._gyro_offset = list(gyro_data)
        self._accel_offset = list(acc_data)
        self._temp_offset = temp_data
        self.get_logger().info("IMU zeroed with ACC: X:%.2f, Y: %.2f, Z: %.2f m/s^2" % acc_data)
        self.get_logger().info("IMU zeroed with Gyro X:%.2f, Y: %.2f, Z: %.2f degrees/s" % gyro_data)
        return []


def main(args=None):
    rclpy.init(args=args)

    node = IMUNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()