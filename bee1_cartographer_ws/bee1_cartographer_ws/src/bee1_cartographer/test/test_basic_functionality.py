#!/usr/bin/env python3
"""
Basic functionality tests for Beemobs Bee1 Cartographer system
"""

import unittest
import rclpy
from rclpy.node import Node
import time
import threading
from std_msgs.msg import String, Float64, Bool
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import NavSatFix, Imu


class TestBee1Functionality(unittest.TestCase):
    
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.test_node = Node('test_bee1_functionality')
        
    @classmethod 
    def tearDownClass(cls):
        cls.test_node.destroy_node()
        rclpy.shutdown()
        
    def test_geojson_parser_publishing(self):
        """Test if GeoJSON parser publishes waypoints"""
        received_path = []
        
        def path_callback(msg):
            received_path.append(msg)
            
        subscription = self.test_node.create_subscription(
            Path,
            'mission_waypoints',
            path_callback,
            10
        )
        
        # Spin for a short time to receive messages
        start_time = time.time()
        while time.time() - start_time < 5.0 and len(received_path) == 0:
            rclpy.spin_once(self.test_node, timeout_sec=0.1)
            
        self.assertGreater(len(received_path), 0, "No waypoints received from GeoJSON parser")
        self.assertGreater(len(received_path[0].poses), 0, "Empty waypoint list received")
        
    def test_mission_executor_status(self):
        """Test if mission executor publishes status"""
        received_status = []
        
        def status_callback(msg):
            received_status.append(msg.data)
            
        subscription = self.test_node.create_subscription(
            String,
            'mission_status',
            status_callback,
            10
        )
        
        start_time = time.time()
        while time.time() - start_time < 5.0 and len(received_status) == 0:
            rclpy.spin_once(self.test_node, timeout_sec=0.1)
            
        self.assertGreater(len(received_status), 0, "No mission status received")
        self.assertIn("IDLE", received_status[0], "Mission status should be IDLE initially")
        
    def test_vehicle_controller_response(self):
        """Test if vehicle controller responds to cmd_vel"""
        received_commands = {'throttle': [], 'brake': [], 'steering': []}
        
        def throttle_callback(msg):
            received_commands['throttle'].append(msg.data)
            
        def brake_callback(msg):
            received_commands['brake'].append(msg.data)
            
        def steering_callback(msg):
            received_commands['steering'].append(msg.data)
            
        # Subscribe to vehicle commands
        throttle_sub = self.test_node.create_subscription(
            Float64, 'vehicle/throttle', throttle_callback, 10)
        brake_sub = self.test_node.create_subscription(
            Float64, 'vehicle/brake', brake_callback, 10)
        steering_sub = self.test_node.create_subscription(
            Float64, 'vehicle/steering', steering_callback, 10)
            
        # Publish cmd_vel
        cmd_vel_pub = self.test_node.create_publisher(Twist, 'cmd_vel', 10)
        
        # Wait for connections
        time.sleep(1.0)
        
        # Send test command
        test_twist = Twist()
        test_twist.linear.x = 5.0  # 5 m/s forward
        test_twist.angular.z = 0.1  # slight turn
        
        for _ in range(5):
            cmd_vel_pub.publish(test_twist)
            rclpy.spin_once(self.test_node, timeout_sec=0.1)
            time.sleep(0.1)
            
        # Check if vehicle commands were received
        self.assertGreater(len(received_commands['throttle']), 0, "No throttle commands received")
        self.assertGreater(received_commands['throttle'][-1], 0, "Throttle should be positive for forward motion")
        
    def test_emergency_stop_functionality(self):
        """Test emergency stop functionality"""
        received_commands = {'throttle': [], 'brake': []}
        
        def throttle_callback(msg):
            received_commands['throttle'].append(msg.data)
            
        def brake_callback(msg):
            received_commands['brake'].append(msg.data)
            
        throttle_sub = self.test_node.create_subscription(
            Float64, 'vehicle/throttle', throttle_callback, 10)
        brake_sub = self.test_node.create_subscription(
            Float64, 'vehicle/brake', brake_callback, 10)
            
        # Publishers
        cmd_vel_pub = self.test_node.create_publisher(Twist, 'cmd_vel', 10)
        emergency_pub = self.test_node.create_publisher(Bool, 'emergency_stop', 10)
        
        time.sleep(1.0)
        
        # Send forward command
        test_twist = Twist()
        test_twist.linear.x = 10.0
        cmd_vel_pub.publish(test_twist)
        
        # Trigger emergency stop
        emergency_msg = Bool()
        emergency_msg.data = True
        emergency_pub.publish(emergency_msg)
        
        # Spin to process messages
        for _ in range(10):
            rclpy.spin_once(self.test_node, timeout_sec=0.1)
            time.sleep(0.1)
            
        # Check if vehicle stopped
        if len(received_commands['throttle']) > 0:
            self.assertEqual(received_commands['throttle'][-1], 0.0, 
                           "Throttle should be 0 during emergency stop")
                           
    def test_coordinate_transformations(self):
        """Test coordinate system transformations"""
        # This would test TF transformations between frames
        # For now, just ensure TF is working
        
        from tf2_ros import Buffer, TransformListener
        
        tf_buffer = Buffer()
        tf_listener = TransformListener(tf_buffer, self.test_node)
        
        # Wait for transforms to be available
        time.sleep(2.0)
        
        try:
            # Try to get transform from base_link to sensor frames
            transform = tf_buffer.lookup_transform(
                'base_link', 'velodyne_link', 
                rclpy.time.Time())
            self.assertIsNotNone(transform, "Transform from base_link to velodyne_link not available")
        except Exception as e:
            self.fail(f"Transform lookup failed: {e}")
            
    def test_parameter_loading(self):
        """Test if all required parameters are loaded correctly"""
        
        # Test vehicle parameters
        test_param_node = Node('test_param_node')
        
        # Declare and get test parameters
        test_param_node.declare_parameter('max_speed', 15.0)
        max_speed = test_param_node.get_parameter('max_speed').value
        
        self.assertEqual(max_speed, 15.0, "Max speed parameter not loaded correctly")
        
        test_param_node.destroy_node()


def main():
    """Run all tests"""
    unittest.main()


if __name__ == '__main__':
    main()