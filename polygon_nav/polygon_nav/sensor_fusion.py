#!/usr/bin/env python3
# sensor_fusion.py
# Fuses data from YOLO, Hand Gesture Detector and other sensors

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Header
import numpy as np
from cv_bridge import CvBridge
import cv2
from copy import deepcopy
from std_msgs.msg import String as RosString

VOICE_ACTION_MAP = {
    'sitz': 'SIT',
    'dog_sit': 'SIT',
    'platz': 'SIT',
    'dog_down': 'SIT',
    'auf': 'UP',
    'up': 'UP',
    'dog_up': 'UP',
    'beifuß': 'FOLLOW',
    'bei fuß': 'FOLLOW',
    'dog_heel': 'FOLLOW',
}

class SensorFusion(Node):
    def __init__(self):
        super().__init__('sensor_fusion')
        
        # Declare parameters
        self.declare_parameter('action_id', 0)
        self.declare_parameter('user_id', -1)  # -1 = unspecified
        
        # Get parameters
        self.action_id = self.get_parameter('action_id').get_parameter_value().integer_value
        self.user_id = self.get_parameter('user_id').get_parameter_value().integer_value
        
        # Data buffers
        self.current_detections = None
        self.current_yolo_image = None
        self.current_hand_finger = None
        self.current_hand_thumbs_up = None
        self.current_hand_image = None
        self.current_voice_command = None
        
        # Subscribers
        self.yolo_detections_sub = self.create_subscription(
            Detection2DArray,
            '/yolo/detections',
            self.yolo_detections_callback,
            1  # Queue size 1
        )
        
        self.yolo_image_sub = self.create_subscription(
            Image,
            '/yolo/processed_image',
            self.yolo_image_callback,
            1  # Queue size 10
        )
        
        self.hand_finger_sub = self.create_subscription(
            PointStamped,
            '/hand/pointer_finger',
            self.hand_finger_callback,
            1  # Queue size 10
        )
        
        self.hand_image_sub = self.create_subscription(
            Image,
            '/hand/annotated_image',
            self.hand_image_callback,
            1  # Queue size 10
        )
        
        self.hand_thumbs_up_sub = self.create_subscription(
            PointStamped,
            '/hand/thumbs_up',
            self.hand_thumbs_up_callback,
            1  # Queue size 10
        )

        self.voice_command_sub = self.create_subscription(
            RosString,
            '/voice_commands',
            self.voice_command_callback,
            1  # Queue size 10
        )

        # Subscribe to state machine to reset tracked_person_id when state changes to IDLE
        self.state_machine_sub = self.create_subscription(
            RosString,
            '/state_machine_out',
            self.state_machine_callback,
            1
        )

        # Publisher
        self.fusion_pub = self.create_publisher(
            RosString,
            '/fusion_out',
            1  # Queue size 10
        )

        self.target_person_pub = self.create_publisher(
            PointStamped,
            '/human_tracker/target_person',
            1
        )


        

        
        # Timer to clear old gestures (1 Hz)
        self.gesture_timeout = 1.0  # Clear gestures after 1.0 seconds (allows for gaps in detection)
        self.finger_timeout = 1.5  # Longer timeout for finger gestures to allow accumulation of duration
        self.last_finger_time = None #time when finger was detected last time
        self.last_thumbs_up_time = None #time when thumbs up was detected last time
        self.create_timer(0.5, self.gesture_timeout_callback) # timer um 
        
        # Gesture duration tracking (3 seconds minimum)
        self.gesture_duration_required = 3.0  # Minimum 3 seconds
        self.thumbs_up_start_time = None  # When thumbs up was first detected
        self.finger_start_time = None  # When finger pointing was first detected
        self.last_published_action = None  # Track last published action to avoid duplicates
        
        # Tracking ID for target person
        self.tracked_person_id = None  # Store the ID of the person being tracked
        
        self.get_logger().info('🔀 Sensor Fusion Node gestartet ✅')
        self.get_logger().info(f'Action ID: {self.action_id}')
        self.get_logger().info(f'User ID: {self.user_id if self.user_id != -1 else "unspecified"}')
        self.get_logger().info('Subscribed to: /yolo/detections, /yolo/processed_image, /hand/pointer_finger, /hand/thumbs_up, /hand/annotated_image')
        self.get_logger().info('Publishing to: /fusion_out')
    
    def yolo_detections_callback(self, msg):
        """Callback für YOLO Detections"""
        self.current_detections = msg
        #function to publish information on /human_tracker/target_person
        self.target_person_merge()
        self.process_and_publish()

    def target_person_merge(self):
        """Hier werden die nötigen Informationen für das follower_tracking published
        Es werden benötigt:
        1. Die Handinformationen von mediapipe
        2. Die Bboxes von yolo mit der richtigen Id die zum Handzeichen gemappt wird.
        
        Die Funktion prüft ob Koordinaten von thumbs_up oder pointer_finger innerhalb
        der Bbox einer YOLO Detection sind. Wenn ja, wird die ID gespeichert und
        solange diese ID vorhanden ist, werden nur Informationen von dieser ID verwendet."""
        
        # Prüfen ob Detections vorhanden sind
        if self.current_detections is None or len(self.current_detections.detections) == 0:
            return
        
        output_point = PointStamped()
        output_point.header.stamp = self.get_clock().now().to_msg()
        # Frame ID aus Detection-Header übernehmen, falls verfügbar
        if hasattr(self.current_detections, 'header') and self.current_detections.header.frame_id:
            output_point.header.frame_id = self.current_detections.header.frame_id
        else:
            output_point.header.frame_id = "camera_frame"  # Fallback
        
        # Wenn eine ID bereits gespeichert ist, nur diese ID verwenden
        if self.tracked_person_id is not None:
            for det in self.current_detections.detections:
                # Prüfen ob Detection die gespeicherte ID hat
                if hasattr(det, 'id') and det.id == self.tracked_person_id:
                    # Informationen aus dieser Detection verwenden
                    output_point.point.x = det.bbox.center.position.x
                    output_point.point.y = det.bbox.size_y  # box_height
                    self.target_person_pub.publish(output_point)
                    self.get_logger().debug(f'📤 Published target_person: center_x={output_point.point.x:.1f}, height={output_point.point.y:.1f} (ID: {self.tracked_person_id})')
                    return
            # Wenn ID nicht mehr vorhanden ist, zurücksetzen
            self.get_logger().info(f'⚠️ Tracked person ID {self.tracked_person_id} not found in detections, resetting')
            self.tracked_person_id = None
        
        # Prüfen ob thumbs_up oder pointer_finger Koordinaten vorhanden sind
        hand_point = None
        if self.current_hand_thumbs_up is not None:
            try:
                hand_point = (self.current_hand_thumbs_up.point.x, self.current_hand_thumbs_up.point.y)
            except Exception:
                pass
        
        if hand_point is None and self.current_hand_finger is not None:
            try:
                hand_point = (self.current_hand_finger.point.x, self.current_hand_finger.point.y)
            except Exception:
                pass
        
        # Wenn Hand-Koordinaten vorhanden sind, prüfen ob sie in einer Bbox liegen
        if hand_point is not None:
            px, py = hand_point
            for det in self.current_detections.detections:
                try:
                    if self.is_point_in_bbox(px, py, det):
                        # ID speichern
                        if hasattr(det, 'id') and det.id:
                            self.tracked_person_id = det.id
                            self.get_logger().info(f'🎯 Person ID {self.tracked_person_id} wird jetzt getrackt')
                        
                        # Informationen aus dieser Detection verwenden
                        output_point.point.x = det.bbox.center.position.x
                        output_point.point.y = det.bbox.size_y  # box_height
                        self.target_person_pub.publish(output_point)
                        return
                except Exception:
                    # Robust gegen fehlerhafte Detections-Felder
                    continue
        
        # Wenn keine Hand-Koordinaten in einer Bbox gefunden wurden, nichts publizieren


    def voice_command_callback(self, msg):
        """Callback für Sprachbefehle"""
        self.current_voice_command = msg.data
        self.get_logger().info(f'🔊 Sprachbefehl empfangen: {msg.data}')
        self.process_and_publish()
    
    def state_machine_callback(self, msg):
        """Callback für State Machine Updates - reset tracked_person_id wenn State zu IDLE wechselt"""
        current_state = msg.data.strip().upper() if msg.data else ""
        # Wenn State von FOLLOW zu IDLE wechselt, ID zurücksetzen
        if current_state == 'IDLE' and self.tracked_person_id is not None:
            self.get_logger().info(f'🛑 State changed to IDLE - Person ID {self.tracked_person_id} wird zurückgesetzt')
            self.tracked_person_id = None    
    
    def yolo_image_callback(self, msg):
        """Callback für YOLO verarbeitetes Bild"""
        self.current_yolo_image = msg
        self.process_and_publish()
    
    def hand_finger_callback(self, msg):
        """Callback für Zeigefinger-Position"""
        current_time = self.get_clock().now()
        self.current_hand_finger = msg
        self.last_finger_time = current_time
        
        # Start tracking finger gesture duration if not already tracking
        if self.finger_start_time is None:
            self.finger_start_time = current_time
            self.get_logger().debug('👆 Finger gesture detected - starting duration tracking')
        
        self.process_and_publish()
    
    def hand_image_callback(self, msg):
        """Callback für Hand-annotiertes Bild"""
        self.current_hand_image = msg
        self.process_and_publish()
    
    def hand_thumbs_up_callback(self, msg):
        """Callback für Thumbs Up Position"""
        current_time = self.get_clock().now()
        self.current_hand_thumbs_up = msg
        self.last_thumbs_up_time = current_time
        
        # Start tracking thumbs up gesture duration if not already tracking
        if self.thumbs_up_start_time is None:
            self.thumbs_up_start_time = current_time
            self.get_logger().debug('👍 Thumbs up gesture detected - starting duration tracking')
        
        self.process_and_publish()
    
    def gesture_timeout_callback(self):
        """Clear gestures if they're older than timeout"""
        now = self.get_clock().now()
        
        # Clear finger if timeout
        if self.last_finger_time:
            time_since_last_finger_detection = (now - self.last_finger_time).nanoseconds / 1e9
            if time_since_last_finger_detection > self.finger_timeout:
                self.current_hand_finger = None
                self.last_finger_time = None
                self.finger_start_time = None  # Reset duration tracking
                self.get_logger().debug('👆 Finger gesture timeout - reset duration tracking')
        
        # Clear thumbs_up if timeout
        if self.last_thumbs_up_time:
            time_since_last_thumbs_up_detection = (now - self.last_thumbs_up_time).nanoseconds / 1e9
            if time_since_last_thumbs_up_detection > self.gesture_timeout:
                self.current_hand_thumbs_up = None
                self.last_thumbs_up_time = None
                self.thumbs_up_start_time = None  # Reset duration tracking
                self.get_logger().debug('👍 Thumbs up gesture timeout - reset duration tracking')
    
    def is_point_in_bbox(self, point_x, point_y, detection):
        """Prüft ob ein Punkt (x,y) innerhalb einer Detection-Bounding-Box liegt"""
        # Bounding Box aus Detection extrahieren
        center_x = detection.bbox.center.position.x
        center_y = detection.bbox.center.position.y
        size_x = detection.bbox.size_x
        size_y = detection.bbox.size_y
        
        # Bounding Box Grenzen berechnen
        x1 = center_x - size_x / 2
        y1 = center_y - size_y / 2
        x2 = center_x + size_x / 2
        y2 = center_y + size_y / 2
        
        # Prüfen ob Punkt innerhalb der Box liegt
        return x1 <= point_x <= x2 and y1 <= point_y <= y2
    
    def process_and_publish(self):
        """Haupt-Logik: Fused Data verarbeiten und publizieren"""
        now = self.get_clock().now()
        result_action = None

        #voice command check
        if self.current_voice_command:
            cmd = str(self.current_voice_command).strip().lower()
            result_action = VOICE_ACTION_MAP.get(cmd)
            if result_action:
                # reset voice command so gestures and detections can trigger actions again
                self.current_voice_command = None

        #thumbs up check
        if result_action is None and self.current_hand_thumbs_up is not None and self.thumbs_up_start_time is not None:
            thumbs_duration = (now - self.thumbs_up_start_time).nanoseconds / 1e9
            if thumbs_duration >= self.gesture_duration_required:
                result_action = "STOP_FOLLOWING"
                # ID zurücksetzen wenn STOP_FOLLOWING ausgelöst wird
                if self.tracked_person_id is not None:
                    self.get_logger().info(f'🛑 STOP_FOLLOWING: Person ID {self.tracked_person_id} wird zurückgesetzt')
                    self.tracked_person_id = None

        #finger pointing check
        if result_action is None and self.current_hand_finger is not None and self.finger_start_time is not None:
            finger_duration = (now - self.finger_start_time).nanoseconds / 1e9
            self.get_logger().debug(f'👆 Finger gesture duration: {finger_duration:.2f}s (required: {self.gesture_duration_required}s)')
            
            if finger_duration >= self.gesture_duration_required and self.current_detections is not None:
                # PointStamped: .point.x / .point.y erwartet
                try:
                    px = self.current_hand_finger.point.x
                    py = self.current_hand_finger.point.y
                    self.get_logger().debug(f'👆 Finger point: ({px:.1f}, {py:.1f})')
                except Exception:
                    px = None
                    py = None
                    self.get_logger().warn('👆 Failed to extract finger coordinates')

                if px is not None and py is not None:
                    found_in_bbox = False
                    for det in self.current_detections.detections:
                        try:
                            if self.is_point_in_bbox(px, py, det):
                                found_in_bbox = True
                                result_action = "FOLLOW"
                                # ID speichern, damit nachfolgend nur noch diese ID verwendet wird
                                if hasattr(det, 'id') and det.id:
                                    self.tracked_person_id = det.id
                                    self.get_logger().info(f'✅ FOLLOW triggered: Finger point ({px:.1f}, {py:.1f}) in bbox of detection {det.id}. Person ID {self.tracked_person_id} wird jetzt getrackt.')
                                else:
                                    self.get_logger().warn('⚠️ FOLLOW triggered but detection has no ID')
                                break
                        except Exception as e:
                            # robust gegen fehlerhafte Detections-Felder
                            self.get_logger().debug(f'Exception checking bbox: {e}')
                            continue
                    
                    if not found_in_bbox:
                        self.get_logger().debug(f'👆 Finger point ({px:.1f}, {py:.1f}) not in any detection bbox. Detections: {len(self.current_detections.detections)}')
                else:
                    self.get_logger().debug('👆 Finger coordinates are None')
            elif self.current_detections is None:
                self.get_logger().debug('👆 No detections available yet')
            else:
                self.get_logger().debug(f'👆 Finger duration not yet reached: {finger_duration:.2f}s < {self.gesture_duration_required}s')
        
        if result_action is None:
            return

        if result_action != self.last_published_action:
            out = RosString()
            out.data = result_action
            self.fusion_pub.publish(out)
            self.last_published_action = result_action


    def check_if_action_publishing_not_needed(self, result_action_id):
        """Check if action needs to be published based on current gestures and durations"""
        return result_action_id is None

    def create_log_for_action(self, result_action_id, detected_count, result_user_id):
        """Create log message for the current action"""
        if result_action_id == "sit":
            self.get_logger().info(
                f'🔀 FUSION: {detected_count} detections, '
                f'THUMBS UP held for 3+ seconds, Action={result_action_id}'
            )
        elif result_action_id == "follow":
            self.get_logger().info(
                f'🔀 FUSION: {detected_count} detections, '
                f'Finger pointing held for 3+ seconds, Action={result_action_id}, User={result_user_id}'
            )
    

def main(args=None):
    rclpy.init(args=args)
    node = SensorFusion()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Sensor Fusion Node beendet 🛑')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

