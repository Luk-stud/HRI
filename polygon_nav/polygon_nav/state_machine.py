#!/usr/bin/env python3
# state_machine.py
# State Machine for Follow/Sit behavior based on sensor fusion output

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import Header
from enum import Enum
from copy import deepcopy

class State(Enum):
    """State Machine States"""
    IDLE = "idle"
    SIT = "sit"
    FOLLOW = "follow"

class StateMachine(Node):
    def __init__(self):
        super().__init__('state_machine')
        
        # Current state
        self.current_state = State.IDLE
        self.current_user_id = None
        self.last_action_id = None  # Track last action to avoid duplicate publications
        
        # Declare parameters
        self.declare_parameter('auto_transition', True)
        self.declare_parameter('debug', False)
        
        self.auto_transition = self.get_parameter('auto_transition').get_parameter_value().bool_value
        self.debug = self.get_parameter('debug').get_parameter_value().bool_value
        
        # Subscriber for fusion output
        self.fusion_sub = self.create_subscription(
            Detection2DArray,
            '/fusion_out',
            self.fusion_callback,
            1  # Queue size 1
        )
        
        # Publisher for state machine output
        self.state_pub = self.create_publisher(
            Detection2DArray,
            '/state_machine_out',
            1  # Queue size 1
        )
        
        # Periodic state check timer
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz
        
        self.get_logger().info('🤖 State Machine Node gestartet ✅')
        self.get_logger().info(f'Initial State: {self.current_state.value.upper()}')
        self.get_logger().info('Subscribed to: /fusion_out')
        self.get_logger().info('Publishing to: /state_machine_out')
        self.log_state()
    
    def fusion_callback(self, msg):
        """Process fusion output and transition states"""
        # Analyze fusion data to determine state
        target_state = self.determine_state(msg)
        
        # State transition if needed - ONLY publish on state change
        if target_state != self.current_state:
            self.transition_to_state(target_state, msg)
            # Publish ONLY when state changes
            self.publish_state(msg)
    
    def determine_state(self, msg):
        """
        Determine target state based on fusion data
        Logic: Follow if action_id="follow", sit if action_id="sit", otherwise keep current
        """
        # Parse action_id from detection.id field (format: "action_id:user_id" or "action_id")
        current_action_id = None
        for detection in msg.detections:
            if detection.id != "":
                # Parse detection.id which contains "action_id" or "action_id:user_id"
                parts = detection.id.split(":")
                current_action_id = parts[0]
                self.get_logger().info(
                    f'📥 State Machine received: detection.id="{detection.id}", '
                    f'parsed action_id="{current_action_id}", '
                    f'last_action_id="{self.last_action_id}"'
                )
                break
        
        # If first detection (last_action_id is None) or action changed
        if current_action_id and current_action_id != self.last_action_id:
            self.last_action_id = current_action_id
            
            self.get_logger().info(
                f'🔍 Detected action_id change: "{self.last_action_id}" → Processing...'
            )
            
            # If action is "follow", transition to FOLLOW state
            if current_action_id == "follow":
                return State.FOLLOW
            # If action is explicitly "sit", transition to SIT state
            elif current_action_id == "sit":
                return State.SIT
            # If action is "default" or "idle", transition to IDLE state
            elif current_action_id == "default" or current_action_id == "idle":
                return State.IDLE
        elif current_action_id == self.last_action_id:
            self.get_logger().info(
                f'⚠️ Same action_id "{current_action_id}" received - no state change'
            )
        
        # Default: Stay in current state (or IDLE if no detections)
        if not msg.detections:
            return State.IDLE
        
        return self.current_state  # Keep current state if no explicit action
    
    def transition_to_state(self, new_state, msg):
        """Perform state transition"""
        old_state = self.current_state
        self.current_state = new_state
        
        self.get_logger().info(
            f'🔄 STATE TRANSITION: {old_state.value.upper()} → {new_state.value.upper()}'
        )
        
        # Extract additional info from message
        if new_state == State.FOLLOW and msg.detections:
            # Extract user_id from detections if available
            for detection in msg.detections:
                if detection.id != "":
                    # Parse format: "action_id:user_id"
                    parts = detection.id.split(":")
                    if len(parts) == 2:
                        try:
                            self.current_user_id = int(parts[1])
                            self.get_logger().info(
                                f'🎯 FOLLOW MODE: Tracking User ID={self.current_user_id}'
                            )
                            break
                        except ValueError:
                            self.current_user_id = None
        elif new_state == State.IDLE:
            # In IDLE mode, no user to track
            self.current_user_id = None
        else:
            # In SIT mode, no user to track
            self.current_user_id = None
        
        self.log_state()
    
    def log_state(self):
        """Log current state information"""
        if self.debug:
            self.get_logger().info(
                f'🤖 Current State: {self.current_state.value.upper()}, '
                f'User ID: {self.current_user_id}'
            )
    
    def publish_state(self, detection_msg):
        """Publish state machine output"""
        # Create output message
        output_msg = Detection2DArray()
        output_msg.header = detection_msg.header
        output_msg.header.stamp = self.get_clock().now().to_msg()
        output_msg.header.frame_id = "state_machine"
        
        # Copy detections and add state information to each detection ID
        for detection in detection_msg.detections:
            new_detection = deepcopy(detection)
            
            # Override the detection ID with state information
            # Format: "state:<state_name>[:user_id]" if user_id exists
            if self.current_user_id is not None:
                new_detection.id = f"{self.current_state.value}:{self.current_user_id}"
            else:
                new_detection.id = self.current_state.value
            
            output_msg.detections.append(new_detection)
        
        # Publish
        self.state_pub.publish(output_msg)
        
        if self.debug:
            self.get_logger().info(
                f'📤 Published: State={self.current_state.value}, '
                f'User={self.current_user_id}, '
                f'Detections={len(output_msg.detections)}'
            )
    
    def timer_callback(self):
        """Periodic callback for state monitoring"""
        # This can be used for timeouts, state resets, etc.
        pass
    
    def get_current_state(self):
        """Get current state as string"""
        return self.current_state.value
    
    def force_state_transition(self, target_state):
        """Force state transition (for testing/debugging)"""
        if isinstance(target_state, str):
            target_state = State(target_state.lower())
        
        self.transition_to_state(target_state, Detection2DArray())
    
    def create_fusion_summary(self):
        """Create summary of current state machine status"""
        return {
            'current_state': self.current_state.value,
            'user_id': self.current_user_id,
            'timestamp': self.get_clock().now()
        }

def main(args=None):
    rclpy.init(args=args)
    node = StateMachine()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('State Machine Node beendet 🛑')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

