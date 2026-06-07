#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

# ToF imports
import board
import busio
import adafruit_vl53l0x

# IR import
from gpiozero import DigitalInputDevice


class ObstacleDetectorNode(Node):
    def __init__(self):
        super().__init__('obstacle_detector_node')
        
        # --- 1. ToF Parameters & Setup ---
        self.declare_parameter('offset_mm', 70)
        self.declare_parameter('i2c_address', 0x29) 
        self.declare_parameter('threshold_mm', 200)
        
        self.tof_sensor = None
        target_address = self.get_parameter('i2c_address').get_parameter_value().integer_value
        
        try:
            self.i2c = busio.I2C(board.SCL, board.SDA)
            self.initialize_tof_sensor(target_address)
        except Exception as e:
            self.get_logger().error(f'ToF Initialization Error: {e}')

        # --- 2. IR Sensor Setup ---
        self.ir_pins = [4, 14, 15, 17]
        self.ir_sensors = {}
        for pin in self.ir_pins:
            self.ir_sensors[pin] = DigitalInputDevice(pin, pull_up=False)
            self.get_logger().info(f'Initialized Active-High IR sensor on GPIO {pin}')

        # --- 3. Publisher & Timer ---
        self.publisher_ = self.create_publisher(Bool, '/obstacle_detected', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def initialize_tof_sensor(self, target_address):
        """Safely connects to the sensor and changes its address if needed."""
        default_address = 0x29
        
        if target_address == default_address:
            self.tof_sensor = adafruit_vl53l0x.VL53L0X(self.i2c, address=default_address)
            self.get_logger().info(f'VL53L0X Initialized at default address {hex(default_address)}.')
            return

        try:
            self.tof_sensor = adafruit_vl53l0x.VL53L0X(self.i2c, address=default_address)
            self.tof_sensor.set_address(target_address)
            self.get_logger().info(f'Successfully changed I2C address to {hex(target_address)}.')
        except ValueError:
            try:
                self.tof_sensor = adafruit_vl53l0x.VL53L0X(self.i2c, address=target_address)
                self.get_logger().info(f'Sensor was already at target address {hex(target_address)}.')
            except ValueError:
                self.get_logger().error(f'Could not find sensor at {hex(default_address)} or {hex(target_address)}.')
                raise

    def timer_callback(self):
        obstacle_detected = False

        # Check ToF Sensor
        if self.tof_sensor is not None:
            try:
                offset = self.get_parameter('offset_mm').get_parameter_value().integer_value
                threshold = self.get_parameter('threshold_mm').get_parameter_value().integer_value
                
                raw_distance = self.tof_sensor.range
                corrected_distance = max(0, raw_distance - offset)
                
                if corrected_distance < threshold:
                    obstacle_detected = True
            except Exception as e:
                self.get_logger().error(f'Error reading ToF sensor: {e}')

        # Check IR Sensors
        for pin, sensor in self.ir_sensors.items():
            # Original logic: sensor outputs 1 (True) when intercepted
            is_intercepted = not bool(sensor.value)
            
            if is_intercepted:
                obstacle_detected = True
                self.get_logger().debug(f'IR Sensor {pin} Intercepted!')

        # Publish the combined state
        msg = Bool()
        msg.data = obstacle_detected
        self.publisher_.publish(msg)
        
        # Optional overall log
        if obstacle_detected:
            self.get_logger().debug('Obstacle detected by at least one sensor!')


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetectorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()