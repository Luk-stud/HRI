#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, Twist
from std_msgs.msg import String
import time

class FollowerControlSkill(Node):
    """
    A skill node for following a human target. It only generates velocity
    commands when the behavior manager sets the state to 'FOLLOWING'.
    """
    def __init__(self):
        super().__init__('follower_control_skill')

        # --- Parameters ---
        self.camera_width = 640.0
        self.desired_box_height = 300.0
        self.max_linear_speed = 0.4
        self.max_angular_speed = 1.0
        self.kp_linear = 0.003
        self.kp_angular = 0.004
        
        self.image_center_x = self.camera_width / 2.0
        self.is_active = False

        # ROS interfaces
        self.create_subscription(String, '/robot_behavior/active_state', self.state_callback, 10)
        self.create_subscription(PointStamped, '/human_tracker/target_person', self.target_callback, 10)
        self.vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info("Follower Control Skill ready.")

    def state_callback(self, msg: String):
        """Activates or deactivates this skill based on the brain's command."""
        # CHANGED: The state name is updated from 'FOLLOWING_PERSON' to 'FOLLOWING'
        # to match the corrected behavior_manager.
        should_be_active = (msg.data == 'FOLLOWING')
        
        if should_be_active and not self.is_active:
            self.get_logger().info("ACTIVATING person following skill.")
        elif not should_be_active and self.is_active:
            self.get_logger().info("DEACTIVATING person following skill.")
            self.publish_stop()
        self.is_active = should_be_active

    def target_callback(self, msg: PointStamped):
        """Calculates and publishes velocity commands based on target position."""
        if not self.is_active:
            return

        center_x = msg.point.x
        box_height = msg.point.y
        
        error_distance = self.desired_box_height - box_height
        error_angle = self.image_center_x - center_x

        linear_vel = self.kp_linear * error_distance
        angular_vel = self.kp_angular * error_angle

        linear_vel = max(min(linear_vel, self.max_linear_speed), -self.max_linear_speed)
        angular_vel = max(min(angular_vel, self.max_angular_speed), -self.max_angular_speed)

        twist = Twist()
        twist.linear.x = linear_vel
        twist.angular.z = angular_vel
        self.vel_publisher.publish(twist)

    def publish_stop(self):
        """Publishes a zero-velocity Twist message to stop the robot."""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.vel_publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = FollowerControlSkill()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
