#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
# Assuming go1_legged_msgs is a custom message type. If not available,
# this would need to be adapted to the correct message type.
from go1_legged_msgs.msg import DesiredPose

class PoseControlSkill(Node):
    """
    A skill node expert in executing body poses and animations.
    It listens to the state published by the behavior manager and acts accordingly.
    """
    def __init__(self):
        super().__init__('pose_control_skill')
        
        self.active_state = None
        self.current_pose = [0.0, 0.0, 0.0, 0.28] # Assume starting at stand
        
        # Animation variables
        self.animation_timer = self.create_timer(0.1, self.animation_step_callback)
        self.animation_timer.cancel()
        self.animation_sequence = []
        self.animation_index = 0
        self.loop_animation = False

        # Data
        self.poses = {
            "SITTING": [0.0, -0.75, 0.0, -0.6],
            "STANDING": [0.0, 0.0, 0.0, 0.28]
        }
        self.dances = {
            # This dance is no longer triggered by the new brain, but we keep the data
            "DANCING_1": [ [0.0, 0.5196, 0.6, 0.0], [0.1396, 0.5155, 0.5571, 0.1376]
            ],
            # CHANGED: A longer, smoother dance sequence designed to loop well.
            # This sequence creates a side-to-side roll motion.
            "DANCING_2": [
                    [0.0, 0.0, 0.0, -0.2],
                    [0.1274, 0.1472, 0.0634, -0.0826],
                    [0.2158, 0.2355, 0.125, 0.0209],
                    [0.2529, 0.2471, 0.182, 0.0941],
                    [0.2399, 0.1809, 0.2317, 0.124],
                    [0.1902, 0.0541, 0.2703, 0.1049],
                    [0.1275, -0.1009, 0.2952, 0.0382],
                    [0.0786, -0.2355, 0.3041, -0.048],
                    [0.0637, -0.313, 0.2954, -0.1254],
                    [0.0882, -0.3132, 0.2686, -0.1768],
                    [0.1414, -0.2349, 0.2245, -0.1914],
                    [0.2002, -0.0971, 0.1646, -0.163],
                    [0.2375, 0.0666, 0.0915, -0.0937],
                    [0.2328, 0.2183, 0.009, 0.0081],
                    [0.1795, 0.3136, -0.0749, 0.1014],
                    [0.0864, 0.3243, -0.1545, 0.1513],
                    [-0.0355, 0.2427, -0.2242, 0.1436],
                    [-0.1577, 0.0828, -0.2788, 0.0818],
                    [-0.2542, -0.1103, -0.3143, -0.0256],
                    [-0.3051, -0.2987, -0.3288, -0.1288],
                    [-0.2997, -0.4441, -0.3216, -0.192],
                    [-0.2389, -0.515, -0.2935, -0.2],
                    [-0.1337, -0.4925, -0.2463, -0.1517],
                    [-0.0035, -0.3742, -0.182, -0.0592],
                    [0.1226, -0.1762, -0.1025, 0.0557],
                    [0.2222, 0.0562, -0.0108, 0.1584],
                    [0.2784, 0.2801, 0.0828, 0.2232],
                    [0.2843, 0.4505, 0.1653, 0.2336],
                    [0.2447, 0.5317, 0.2245, 0.1842],
                    [0.1748, 0.501, 0.2514, 0.0801],
                    [0.0973, 0.3549, 0.2414, -0.0465],
                    [0.0352, 0.1091, 0.196, -0.1581],
                    [-0.0015, -0.1634, 0.1226, -0.2221],
                    [-0.0102, -0.4005, 0.0322, -0.2206],
                    [0.0112, -0.5495, -0.0634, -0.1515],
                    [0.0609, -0.5797, -0.151, -0.0272],
                    [0.1322, -0.4854, -0.2187, 0.1071],
                    [0.2155, -0.2861, -0.2588, 0.2163],
                    [0.2995, -0.0237, -0.2682, 0.2716],
                    [0.3716, 0.2467, -0.2486, 0.2572],
                    [0.4204, 0.4635, -0.2061, 0.1718],
                    [0.4363, 0.5742, -0.1507, 0.0299],
                    [0.4125, 0.5446, -0.0943, -0.1311],
                    [0.3456, 0.3673, -0.0487, -0.2608],
                    [0.2366, 0.0647, -0.0236, -0.3075],
                    [0.0937, -0.2702, -0.0256, -0.2533],
                    [-0.0662, -0.5594, -0.0563, -0.1202],
                    [-0.225, -0.7347, -0.1116, 0.0411],
                    [-0.3652, -0.7505, -0.1836, 0.1788],
                    [-0.4725, -0.5926, -0.2608, 0.2385],
                    [-0.5376, -0.2841, -0.3304, 0.2007],
            ]
        }
        self.idle_sequence = [
            [0.0, 0.14, 0.0, 0.28], [0.08, 0.13, 0.0, 0.27], [0.15, 0.11, 0.0, 0.27],
            [0.21, 0.08, 0.0, 0.26], [0.25, 0.04, 0.0, 0.26], [0.27, 0.0, 0.0, 0.26],
            [0.27, -0.04, 0.0, 0.27], [0.25, -0.08, 0.0, 0.27], [0.21, -0.11, 0.0, 0.28],
            [0.15, -0.13, 0.0, 0.28], [0.0, -0.14, 0.0, 0.28], [-0.08, -0.13, 0.0, 0.27],
            [-0.15, -0.11, 0.0, 0.27], [-0.21, -0.08, 0.0, 0.26], [-0.25, -0.04, 0.0, 0.26],
            [-0.27, 0.0, 0.0, 0.26], [-0.27, 0.04, 0.0, 0.27], [-0.25, 0.08, 0.0, 0.27],
            [-0.21, 0.11, 0.0, 0.28], [-0.15, 0.13, 0.0, 0.28]
        ]

        self.pose_publisher = self.create_publisher(DesiredPose, '/desired_pose', 10)
        self.create_subscription(String, '/robot_behavior/active_state', self.state_callback, 10)
        self.get_logger().info("Pose Control Skill ready.")

    def state_callback(self, msg: String):
        """Responds to commands from the behavior manager."""
        new_state = msg.data
        if new_state == self.active_state:
            return # No change
        
        self.get_logger().info(f"New command from brain: '{new_state}'")
        self.active_state = new_state
        self.animation_timer.cancel() # Stop any previous animation

        if self.active_state == 'IDLE':
            self.start_animation(self.idle_sequence, loop=True)

        # NOTE: SITTING is no longer a state in the new FSM, but the logic is kept here.
        elif self.active_state == 'SITTING':
            self.start_interpolation(self.poses['SITTING'])

        # CHANGED: Now triggers the dance animation with looping enabled.
        elif self.active_state == 'DANCING':
            self.start_animation(self.dances["DANCING_2"], loop=True)

        # CHANGED: State name updated from 'FOLLOWING_PERSON' to 'FOLLOWING'.
        elif self.active_state == 'FOLLOWING':
            # When following, transition smoothly back to a neutral standing pose
            self.start_interpolation(self.poses['STANDING'])

    def start_interpolation(self, target_pose, duration=1.5, steps=15):
        """Generates a smooth transition from the current pose to a target pose."""
        sequence = []
        start_pose = self.current_pose
        for i in range(steps + 1):
            progress = float(i) / steps
            interp_pose = [
                start_pose[j] + (target_pose[j] - start_pose[j]) * progress
                for j in range(4)
            ]
            sequence.append(interp_pose)
        self.start_animation(sequence)

    def start_animation(self, sequence, loop=False):
        """Starts playing a sequence of poses."""
        self.animation_sequence = sequence
        self.animation_index = 0
        self.loop_animation = loop
        self.animation_timer.reset()

    def animation_step_callback(self):
        """Callback for the animation timer, publishes one pose frame."""
        if not self.animation_sequence:
            return

        if self.animation_index >= len(self.animation_sequence):
            if self.loop_animation:
                self.animation_index = 0
            else:
                self.animation_timer.cancel()
                return
        
        pose_values = self.animation_sequence[self.animation_index]
        self.current_pose = pose_values # Update current pose
        
        pose_msg = DesiredPose()
        pose_msg.roll, pose_msg.pitch, pose_msg.yaw, pose_msg.body_height = pose_values
        self.pose_publisher.publish(pose_msg)
        
        self.animation_index += 1

def main(args=None):
    rclpy.init(args=args)
    node = PoseControlSkill()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

