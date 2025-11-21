#!/usr/bin/env python3
# sit_node.py
# Controls robot to sit position when sit state is received
# Uses SportClient SDK if available, otherwise falls back to ROS2 SetMode service

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from vision_msgs.msg import Detection2D
from std_msgs.msg import Header
from go1_legged_msgs.srv import SetMode
import time

# Try to import robot_interface from Unitree SDK
SDK_AVAILABLE = False
sdk = None

try:
    import sys
    import os
    # Add SDK Python library path
    sdk_path = os.path.join(os.path.expanduser('~'), 'ROS2', 'unitree_legged_sdk', 'lib', 'python', 'amd64')
    if os.path.exists(sdk_path):
        sys.path.append(sdk_path)
        import robot_interface as sdk
        SDK_AVAILABLE = True
    else:
        # Try arm64 path
        sdk_path = os.path.join(os.path.expanduser('~'), 'ROS2', 'unitree_legged_sdk', 'lib', 'python', 'arm64')
        if os.path.exists(sdk_path):
            sys.path.append(sdk_path)
            import robot_interface as sdk
            SDK_AVAILABLE = True
except ImportError:
    SDK_AVAILABLE = False
except Exception as e:
    SDK_AVAILABLE = False

class SitNode(Node):
    def __init__(self):
        super().__init__('sit_node')
        
        # Try to initialize SDK if available
        self.sdk_udp = None
        self.sdk_cmd = None
        self.sdk_state = None
        self.use_sdk = False
        
        if SDK_AVAILABLE and sdk is not None:
            try:
                # Initialize SDK UDP connection
                HIGHLEVEL = 0xee
                # Default robot IP (can be configured via parameter)
                robot_ip = self.declare_parameter('robot_ip', '192.168.123.161').value
                self.sdk_udp = sdk.UDP(HIGHLEVEL, 8080, robot_ip, 8082)
                self.sdk_cmd = sdk.HighCmd()
                self.sdk_state = sdk.HighState()
                self.sdk_udp.InitCmdData(self.sdk_cmd)
                
                # Initialize command with default values
                self.sdk_cmd.mode = 0  # idle, default stand
                self.sdk_cmd.gaitType = 0
                self.sdk_cmd.speedLevel = 0
                self.sdk_cmd.footRaiseHeight = 0
                self.sdk_cmd.bodyHeight = 0
                self.sdk_cmd.euler = [0, 0, 0]
                self.sdk_cmd.velocity = [0, 0]
                self.sdk_cmd.yawSpeed = 0.0
                self.sdk_cmd.reserve = 0
                
                self.use_sdk = True
                self.get_logger().info(f'✅ Unitree SDK initialized - using SDK for robot control (IP: {robot_ip})')
            except Exception as e:
                self.get_logger().warn(f'⚠️ Failed to initialize SDK: {e} - falling back to ROS2 Service')
                self.use_sdk = False
        
        # Service client for SetMode (fallback if SDK not available)
        self.set_mode_client = None
        if not self.use_sdk:
            self.set_mode_client = self.create_client(SetMode, '/set_mode')
            
            # Wait for service to be available
            while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn('⏳ Waiting for /set_mode service...')
            
            self.get_logger().info('✅ SetMode service client initialized (fallback mode)')
        
        # Subscriber for state machine output
        self.state_sub = self.create_subscription(
            Detection2DArray,
            '/state_machine_out',
            self.state_callback,
            1  # Queue size 1
        )
        
        # Publisher to fusion_out to trigger state machine transition to idle
        self.fusion_pub = self.create_publisher(
            Detection2DArray,
            '/fusion_out',
            1  # Queue size 1
        )
        
        # Current state
        self.current_state = None
        self.is_sitting = False
        self.sit_start_time = None
        self.sit_duration = 10.0  # 10 seconds
        self.should_publish_idle = False  # Flag to continuously publish idle
        self.idle_publish_count = 0  # Count how many times we've published idle
        self.max_idle_publishes = 20  # Publish idle for max 2 seconds (20 * 0.1s)
        
        # For delayed mode 0 call after stand up
        self.pending_mode_0_call = False
        self.mode_0_call_time = None
        self.mode_0_delay = 2.0  # 2 seconds delay after stand up (robot needs time to complete movement)
        
        # Timer for periodic checking and idle state publishing
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz
        
        # Timer for SDK command sending (SDK requires continuous commands)
        if self.use_sdk:
            self.sdk_timer = self.create_timer(0.002, self.sdk_timer_callback)  # 500 Hz (as per SDK examples)
        
        self.get_logger().info('🤖 Sit Node gestartet ✅')
        self.get_logger().info('Subscribed to: /state_machine_out')
        self.get_logger().info('Publishing to: /fusion_out (for idle state)')
        if self.use_sdk:
            self.get_logger().info('Using: Unitree SDK (robot_interface - cmd.mode)')
        else:
            self.get_logger().info('Using: ROS2 Service /set_mode (mode 5=sit, mode 6=stand)')
    
    def state_callback(self, msg):
        """Process state machine output"""
        # Parse state from detections
        state = None
        for detection in msg.detections:
            if detection.id != "":
                parts = detection.id.split(":")
                state = parts[0]
                break
        
        self.get_logger().info(
            f'📥 Sit Node received state: "{state}", currently sitting: {self.is_sitting}'
        )
        
        # If state is "sit" and we're not already sitting, start sit sequence
        if state == "sit":
            current_time = self.get_clock().now().nanoseconds / 1e9
            if not self.is_sitting:
                self.get_logger().info('🪑 Starting sit sequence...')
                self.is_sitting = True
                self.sit_start_time = current_time
                self.should_publish_idle = False  # Reset idle flag for new sit cycle
                self.idle_publish_count = 0  # Reset counter
                
                # Call sit command (SDK or Service)
                self.execute_sit()
            # else: already sitting, just update time tracking
        elif state == "idle" and self.should_publish_idle:
            # State machine has transitioned to idle, stop publishing idle
            self.get_logger().info('✅ State machine transitioned to IDLE, stopping idle publishes')
            self.should_publish_idle = False
            self.idle_publish_count = 0
        elif state != "sit" and self.is_sitting:
            # Return to normal position if state changed away from sit
            self.get_logger().info('🔄 State changed away from sit, returning to normal position...')
            self.return_to_normal()
            # Stop publishing idle if we were
            self.should_publish_idle = False
            self.idle_publish_count = 0
    
    def sdk_timer_callback(self):
        """Periodic callback to send SDK commands (SDK requires continuous command stream)"""
        if self.use_sdk and self.sdk_udp is not None and self.sdk_cmd is not None:
            try:
                # Receive state (non-blocking)
                self.sdk_udp.Recv()
                # Send command
                self.sdk_udp.SetSend(self.sdk_cmd)
                self.sdk_udp.Send()
            except Exception as e:
                # Only log errors occasionally to avoid spam
                pass
    
    def timer_callback(self):
        """Periodic callback to check sit duration and publish idle state"""
        current_time = self.get_clock().now()
        current_time_sec = current_time.nanoseconds / 1e9
        
        # Handle delayed mode 0 call after stand up
        if self.pending_mode_0_call and self.mode_0_call_time is not None:
            elapsed = (current_time - self.mode_0_call_time).nanoseconds / 1e9
            if elapsed >= self.mode_0_delay:
                if self.use_sdk:
                    self.get_logger().info('📞 Calling SDK mode=0 (idle/default stand - normal operation)')
                    self.execute_idle()
                else:
                    self.get_logger().info('📞 Calling SetMode mode=0 (idle/default stand - normal operation)')
                    # Use async for mode 0 as well - more reliable
                    self.call_set_mode(0, wait_for_result=False)  # mode 0 = idle, default stand (normal operation)
                self.pending_mode_0_call = False
                self.mode_0_call_time = None
                self.mode_0_delay = 2.0  # Reset delay
                self.get_logger().info('✅ Returned to normal position and operation mode (mode 0 sent)')
        
        if self.is_sitting and self.sit_start_time is not None:
            sit_elapsed = current_time_sec - self.sit_start_time
            
            # Check if we've been sitting for 10 seconds
            if sit_elapsed >= self.sit_duration:
                if not self.should_publish_idle:
                    self.get_logger().info(
                        f'⏰ Sit duration ({self.sit_duration}s) complete, returning to normal position'
                    )
                    self.return_to_normal()
                    # Start publishing idle messages
                    self.should_publish_idle = True
                    self.idle_publish_count = 0
        # Continuously publish idle if needed (for up to 2 seconds)
        if self.should_publish_idle and self.idle_publish_count < self.max_idle_publishes:
            self.publish_idle_state()
            self.idle_publish_count += 1
        elif self.idle_publish_count >= self.max_idle_publishes:
            # Stop publishing after max attempts
            self.should_publish_idle = False
            self.get_logger().warn('⚠️ Stopped publishing idle after max attempts')
    
    def execute_sit(self):
        """Execute sit command using SDK or Service"""
        if self.use_sdk and self.sdk_udp is not None and self.sdk_cmd is not None:
            try:
                # Set mode to 5 (sit)
                self.sdk_cmd.mode = 5
                self.sdk_udp.SetSend(self.sdk_cmd)
                self.sdk_udp.Send()
                self.get_logger().info('✅ Called SDK: cmd.mode = 5 (sit)')
            except Exception as e:
                self.get_logger().error(f'❌ Error calling SDK sit: {e}')
        else:
            # Fallback to ROS2 Service
            self.call_set_mode(5)
    
    def execute_stand(self):
        """Execute stand up command using SDK or Service"""
        if self.use_sdk and self.sdk_udp is not None and self.sdk_cmd is not None:
            try:
                # Set mode to 6 (stand up)
                self.sdk_cmd.mode = 6
                self.sdk_udp.SetSend(self.sdk_cmd)
                self.sdk_udp.Send()
                self.get_logger().info('✅ Called SDK: cmd.mode = 6 (stand up)')
            except Exception as e:
                self.get_logger().error(f'❌ Error calling SDK stand: {e}')
        else:
            # Fallback to ROS2 Service
            self.call_set_mode(6, wait_for_result=False)
    
    def execute_idle(self):
        """Execute idle command using SDK (return to normal operation)"""
        if self.use_sdk and self.sdk_udp is not None and self.sdk_cmd is not None:
            try:
                # Set mode to 0 (idle, default stand)
                self.sdk_cmd.mode = 0
                self.sdk_udp.SetSend(self.sdk_cmd)
                self.sdk_udp.Send()
                self.get_logger().info('✅ Called SDK: cmd.mode = 0 (idle)')
            except Exception as e:
                self.get_logger().error(f'❌ Error calling SDK idle: {e}')
    
    def call_set_mode(self, mode, gait_type=0, wait_for_result=False):
        """
        Call SetMode service to change robot mode
        
        SetMode sind Betriebsmodi des Roboters:
        - Mode 0: idle, default stand (normal operation)
        - Mode 5: position stand down (SIT)
        - Mode 6: position stand up (STAND)
        - Mode 1: force stand (controlled)
        - Mode 2: target velocity walking
        - Mode 3: target position walking
        """
        request = SetMode.Request()
        request.mode = mode
        request.gait_type = gait_type
        
        if wait_for_result:
            # Synchronous call - wait for result with longer timeout
            self.get_logger().info(f'📞 Calling SetMode service (sync): mode={mode}, gait_type={gait_type}')
            try:
                future = self.set_mode_client.call_async(request)
                # Increased timeout to 5 seconds - robot movements can take time
                rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
                if future.done():
                    try:
                        response = future.result()
                        if response.success:
                            self.get_logger().info(f'✅ SetMode service succeeded: mode={mode}')
                        else:
                            self.get_logger().warn(f'⚠️ SetMode service returned success=False: mode={mode}')
                    except Exception as e:
                        self.get_logger().warn(f'⚠️ SetMode service result error: {e} - but command may have been sent')
                else:
                    # Timeout - but the command might still have been sent
                    self.get_logger().warn(
                        f'⚠️ SetMode service call timeout after 5s: mode={mode} '
                        f'(command may still have been processed by robot)'
                    )
            except Exception as e:
                self.get_logger().error(f'❌ SetMode service call failed: {e}')
        else:
            # Asynchronous call - fire and forget
            future = self.set_mode_client.call_async(request)
            self.get_logger().info(f'📞 Called SetMode service (async): mode={mode}, gait_type={gait_type}')
        
        return future
    
    def return_to_normal(self):
        """Return robot to normal standing position"""
        self.get_logger().info('🔄 Starting return to normal sequence...')
        
        if self.use_sdk:
            # Using SDK - first stand up, then set to idle
            self.execute_stand()
            # Schedule idle mode call after delay (using timer callback)
            self.pending_mode_0_call = True
            self.mode_0_call_time = self.get_clock().now()
            self.mode_0_delay = 2.0  # 2 seconds delay after stand up
            self.is_sitting = False
            self.sit_start_time = None
            self.get_logger().info('✅ Initiated return to normal position (using SDK - mode 6 sent, mode 0 will follow)')
        else:
            # Using ROS2 Service - need mode 6 then mode 0
            # First stand up (mode 6 = position stand up)
            # Use async call - even if service doesn't respond, the command is sent
            self.call_set_mode(6, wait_for_result=False)
            self.get_logger().info('📞 Sent SetMode mode=6 (stand up) - async')
            
            # Small delay to let stand up complete (using ROS2 timer instead of blocking sleep)
            # We'll use a flag and timer callback to handle the second call
            self.pending_mode_0_call = True
            self.mode_0_call_time = self.get_clock().now()
            # Increase delay to 2 seconds to give robot time to stand up
            self.mode_0_delay = 2.0
            
            self.is_sitting = False
            self.sit_start_time = None
            self.get_logger().info('✅ Initiated return to normal position (mode 6 sent, mode 0 will follow in 2s)')
    
    def publish_idle_state(self):
        """Publish idle state to /fusion_out to trigger state machine transition"""
        # Create Detection2DArray message with idle action_id
        idle_msg = Detection2DArray()
        idle_msg.header = Header()
        idle_msg.header.stamp = self.get_clock().now().to_msg()
        idle_msg.header.frame_id = "sit_node"
        
        # Create a detection with action_id="idle"
        detection = Detection2D()
        detection.id = "idle"  # This tells state machine to transition to IDLE state
        idle_msg.detections.append(detection)
        
        # Publish to fusion_out (which state machine listens to)
        self.fusion_pub.publish(idle_msg)
        
        if self.idle_publish_count == 1:
            self.get_logger().info(
                '📤 Publishing idle state to /fusion_out to transition state machine to IDLE'
            )

def main(args=None):
    rclpy.init(args=args)
    node = SitNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Sit Node beendet 🛑')
    finally:
        # Return to normal position before shutting down
        if node.is_sitting:
            node.return_to_normal()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
