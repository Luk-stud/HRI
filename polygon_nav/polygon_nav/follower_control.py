#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, Twist
from std_msgs.msg import String


class PIDController:
    """Simple PID controller with optional output and integral limits."""

    def __init__(self, kp, ki, kd, output_limits=None, integral_limit=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limits = output_limits  # tuple (min, max)
        self.integral_limit = integral_limit
        self.integral = 0.0
        self.prev_error = None

    def reset(self):
        self.integral = 0.0
        self.prev_error = None

    def update(self, error, dt):
        derivative = 0.0

        if dt and dt > 0.0:
            self.integral += error * dt
            if self.integral_limit is not None:
                min_int, max_int = self.integral_limit
                if min_int is not None:
                    self.integral = max(min_int, self.integral)
                if max_int is not None:
                    self.integral = min(max_int, self.integral)
            if self.prev_error is not None:
                derivative = (error - self.prev_error) / dt
        else:
            # No time delta -> skip integral growth and derivative term
            derivative = 0.0

        self.prev_error = error

        output = (
            self.kp * error +
            self.ki * self.integral +
            self.kd * derivative
        )

        if self.output_limits is not None:
            min_out, max_out = self.output_limits
            if min_out is not None:
                output = max(min_out, output)
            if max_out is not None:
                output = min(max_out, output)

        return output

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
        self.max_linear_speed = 0.5
        self.max_angular_speed = 1
        self.target_timeout = self.declare_parameter('target_timeout', 1.0).value
        self.kp_linear = 0.005
        self.ki_linear = 0.0005
        self.kd_linear = 0.0001
        self.kp_angular = 0.004
        self.ki_angular = 0.00
        self.kd_angular = 0.001
        
        self.image_center_x = self.camera_width / 2.0
        self.is_active = False
        self.last_target_time = None
        self.last_detection_time = None
        self.target_lost = True

        # ROS interfaces
        self.create_subscription(String, '/state_machine_out', self.state_callback, 10)
        self.create_subscription(PointStamped, '/human_tracker/target_person', self.target_callback, 10)
        
        self.vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.timeout_timer = self.create_timer(0.1, self.monitor_target_timeout)

        # PID controllers for linear (distance) and angular (heading) control
        self.linear_pid = PIDController(
            kp=self.kp_linear,
            ki=self.ki_linear,
            kd=self.kd_linear,
            output_limits=(0.0, self.max_linear_speed),
            integral_limit=(None, self.max_linear_speed)
        )
        self.angular_pid = PIDController(
            kp=self.kp_angular,
            ki=self.ki_angular,
            kd=self.kd_angular,
            output_limits=(-self.max_angular_speed, self.max_angular_speed),
            integral_limit=(None, None)
        )
        
        self.get_logger().info("Follower Control Skill ready.")
        
    def state_callback(self, msg: String):
        """Activates or deactivates this skill based on the brain's command."""
        state = msg.data.strip().upper() if msg.data else ""
        should_be_active = (state == 'FOLLOW')
        
        if should_be_active and not self.is_active:
            self.get_logger().info("ACTIVATING person following skill.")
            self.reset_controllers()
        elif not should_be_active and self.is_active:
            self.get_logger().info("DEACTIVATING person following skill.")
            self.publish_stop()
            self.reset_controllers()
        self.is_active = should_be_active
    
    def reset_controllers(self):
        """Reset PID memory to avoid stale integral/derivative terms."""
        self.linear_pid.reset()
        self.angular_pid.reset()
        self.last_target_time = None
        self.last_detection_time = None
        self.target_lost = True

    def target_callback(self, msg: PointStamped):
        """Calculates and publishes velocity commands based on target position."""
        if not self.is_active:
            self.get_logger().debug("Target received but follower is not active")
            return

        center_x = msg.point.x
        box_height = msg.point.y
        self.get_logger().debug(f"Target received: center_x={center_x:.1f}, box_height={box_height:.1f}")
        
        now = self.get_clock().now().nanoseconds / 1e9
        dt = None if self.last_target_time is None else max(0.0, now - self.last_target_time)
        self.last_target_time = now
        self.last_detection_time = now
        if self.target_lost:
            self.get_logger().info("Target reacquired, resuming motion.")
            self.target_lost = False

        error_distance = self.desired_box_height - box_height
        error_angle = self.image_center_x - center_x

        linear_vel = self.linear_pid.update(error_distance, dt)
        angular_vel = self.angular_pid.update(error_angle, dt)

        twist = Twist()
        twist.linear.x = float(linear_vel)
        twist.angular.z = float(angular_vel)
        self.vel_publisher.publish(twist)

    def publish_stop(self):
        """Publishes a zero-velocity Twist message to stop the robot."""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.vel_publisher.publish(twist)

    def monitor_target_timeout(self):
        """Stop the robot if no target updates arrive within timeout."""
        if not self.is_active:
            return
        if self.last_detection_time is None:
            return
        now = self.get_clock().now().nanoseconds / 1e9
        if (now - self.last_detection_time) > self.target_timeout and not self.target_lost:
            self.get_logger().warn("Target lost - stopping robot until detections resume.")
            self.publish_stop()
            self.linear_pid.reset()
            self.angular_pid.reset()
            self.target_lost = True

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
