#!/usr/bin/env python3
# follow_node.py
# Makes robot follow commands based on state machine output

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class FollowNode(Node):
    def __init__(self):
        super().__init__('follow_node')
        
        # Subscriber for state machine output
        self.state_sub = self.create_subscription(
            Detection2DArray,
            '/state_machine_out',
            self.state_callback,
            1  # Queue size 1
        )
        
        # Subscriber for odometry (to track movement)
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            1  # Queue size 1
        )
        
        # Publisher for robot movement
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            1  # Queue size 1
        )
        
        # State tracking
        self.current_odom = None
        self.moving = False
        self.start_position = None
        self.target_distance = 0.5  # 50 cm
        self.move_speed = 0.2  # m/s
        
        self.get_logger().info('🤖 Follow Node gestartet ✅')
        self.get_logger().info('Subscribed to: /state_machine_out, /odom')
        self.get_logger().info('Publishing to: /cmd_vel')
        self.get_logger().info(f'Target distance: {self.target_distance}m')
    
    def odom_callback(self, msg):
        """Store current odometry"""
        self.current_odom = msg
    
    def state_callback(self, msg):
        """Process state machine output"""
        # Parse action_id from detections
        action_id = None
        user_id = None
        
        for detection in msg.detections:
            if detection.id != "":
                parts = detection.id.split(":")
                action_id = parts[0]
                if len(parts) > 1:
                    try:
                        user_id = int(parts[1])
                    except ValueError:
                        user_id = None
                break
        
        self.get_logger().info(
            f'📥 Follow Node received: action_id="{action_id}", user_id={user_id}, moving={self.moving}'
        )
        
        if action_id == "follow" and not self.moving:
            self.get_logger().info(f'🚶 Starting follow command (user_id={user_id})')
            self.start_follow_sequence()
        elif action_id == "sit" and self.moving:
            self.get_logger().info('🛑 STOP command received')
            self.stop_robot()
    
    def start_follow_sequence(self):
        """Start moving forward 50cm"""
        if self.current_odom is None:
            self.get_logger().warn('No odometry data yet, cannot start movement')
            return
        
        # Record start position
        self.start_position = self.current_odom.pose.pose.position
        
        self.moving = True
        self.get_logger().info(
            f'📍 Starting position: x={self.start_position.x:.2f}, y={self.start_position.y:.2f}'
        )
    
    def stop_robot(self):
        """Stop the robot"""
        self.moving = False
        self.start_position = None
        
        # Publish zero velocity to stop
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
    
    def timer_callback(self):
        """Main control loop - runs at 10 Hz"""
        if not self.moving or self.start_position is None:
            return
        
        if self.current_odom is None:
            return
        
        # Calculate distance moved
        current_pos = self.current_odom.pose.pose.position
        dx = current_pos.x - self.start_position.x
        dy = current_pos.y - self.start_position.y
        distance_moved = math.sqrt(dx*dx + dy*dy)
        
        # Check if we've moved 50cm
        if distance_moved >= self.target_distance:
            self.get_logger().info(
                f'✅ Reached target distance: {distance_moved:.2f}m'
            )
            self.stop_robot()
            return
        
        # Calculate remaining distance
        remaining_distance = self.target_distance - distance_moved
        
        # Publish velocity command
        cmd = Twist()
        
        if remaining_distance > 0.1:  # More than 10cm to go
            # Move at full speed
            cmd.linear.x = self.move_speed
        elif remaining_distance > 0.05:  # 5-10cm to go
            # Slow down for precision
            cmd.linear.x = self.move_speed * 0.5
        else:
            # Almost there, very slow
            cmd.linear.x = self.move_speed * 0.2
        
        cmd.linear.x = min(cmd.linear.x, remaining_distance * 2.0)  # Don't overshoot
        
        # Publish command
        self.cmd_vel_pub.publish(cmd)
        
        # Log progress
        self.get_logger().info(
            f'🚶 Moving: {distance_moved:.3f}m / {self.target_distance:.3f}m '
            f'({remaining_distance:.3f}m remaining)'
        )
    
    def setup_timer(self):
        """Create periodic timer for control loop"""
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz

def main(args=None):
    rclpy.init(args=args)
    node = FollowNode()
    node.setup_timer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Follow Node beendet 🛑')
    finally:
        # Emergency stop
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

