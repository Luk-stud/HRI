#!/usr/bin/env python3
# state_machine.py
# State Machine for Follow/Sit behavior based on sensor fusion output

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import Header
from enum import Enum
from copy import deepcopy
from std_msgs.msg import String as RosString

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
        self.last_action_id = None  # Track last action to avoid duplicate publications
        
        # Declare parameters
        self.declare_parameter('debug', False)
        self.debug = self.get_parameter('debug').get_parameter_value().bool_value
        self.last_published_state = None 
        
        # Subscriber for fusion output
        self.fusion_sub = self.create_subscription(
            RosString,
            '/fusion_out',
            self.fusion_callback,
            1  # Queue size 1
        )
        
        # Publisher for state machine output
        self.state_pub = self.create_publisher(
            RosString,
            '/state_machine_out',
            1  # Queue size 1
        )
        
        
        self.get_logger().info('🤖 State Machine Node gestartet ✅')
        self.get_logger().info(f'Initial State: {self.current_state.value.upper()}')
        self.get_logger().info('Subscribed to: /fusion_out')
        self.get_logger().info('Publishing to: /state_machine_out')
        self.log_state()
    
    def fusion_callback(self, msg):
        """Process fusion output and transition states"""
        action_str = msg.data.strip() if hasattr(msg, 'data') else ''
        if not action_str:
            return
        # Übergabe als normalisierte Großschreibungs-Variante
        self.transition_to_state(action_str.strip().upper())

    
    
    def transition_to_state(self, incoming_action_str: str):
        """Apply transition rules based on current_state and incoming action string.
        Erwartete incoming_action_str: 'FOLLOW','STOP_FOLLOWING','SIT','UP' (Großschreibung empfohlen).
        Übergangsregeln:
          IDLE + FOLLOW -> FOLLOW (publish "FOLLOW")
          IDLE + SIT -> SIT (publish "SIT")
          FOLLOW + STOP_FOLLOWING -> IDLE (publish "IDLE")
          SIT + UP -> IDLE (publish "IDLE")
        """
        incoming = str(incoming_action_str).strip().upper()
        old_state = self.current_state
        target_state = None

        if self.current_state == State.IDLE:
            if incoming == "FOLLOW":
                target_state = State.FOLLOW
            elif incoming == "SIT":
                target_state = State.SIT
        elif self.current_state == State.FOLLOW:
            if incoming == "STOP_FOLLOWING":
                target_state = State.IDLE
        elif self.current_state == State.SIT:
            if incoming == "UP":
                target_state = State.IDLE
        if target_state is None or target_state == self.current_state:
            return

        # Apply transition
        self.current_state = target_state
        self.get_logger().info(f'🔄 STATE TRANSITION: {old_state.value.upper()} → {self.current_state.value.upper()}')
        self.log_state()
        # publish new state as simple string
        self.publish_state()
        
    
    def log_state(self):
        """Log current state information"""
        if self.debug:
            self.get_logger().info(f'🤖 Current State: {self.current_state.value.upper()}')
    
    def publish_state(self):
        """Publish current state as std_msgs/String on /state_machine_out
        and avoid duplicate publications."""
        out = RosString()
        out.data = self.current_state.value.upper()
        if out.data == self.last_published_state:
            return  # avoid duplicate publications
        self.state_pub.publish(out)
        self.last_published_state = out.data
        if self.debug:
            self.get_logger().info(f'📤 Published: {out.data}')
    
    def get_current_state(self):
        """Get current state as string"""
        return self.current_state.value
    
    def create_fusion_summary(self):
        """Create summary of current state machine status"""
        return {
            'current_state': self.current_state.value,
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

