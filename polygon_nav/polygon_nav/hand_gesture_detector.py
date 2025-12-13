#!/usr/bin/env python3
# hand_gesture_detector.py
# MediaPipe Hand-Tracking Node für Zeigefinger-Erkennung

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import cv2
import mediapipe as mp
import math
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class HandGestureDetector(Node):
    def __init__(self):
        super().__init__('hand_gesture_detector')
        
        # MediaPipe Hand-Lösung
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=2,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )
        self.mp_draw = mp.solutions.drawing_utils
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Subscriber für Bildstream
        qos_profile = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST
        )
        self.image_sub = self.create_subscription(
            Image,
            '/image',  # Input topic
            self.image_callback,
            qos_profile
        )
        
        # Publisher für Zeigefinger-Position
        self.finger_pub = self.create_publisher(
            PointStamped,
            '/hand/pointer_finger',  # Output topic
            1  # Queue size 1
        )
        
        # Publisher für Thumbs Up Position
        self.thumbs_up_pub = self.create_publisher(
            PointStamped,
            '/hand/thumbs_up',  # Output topic for thumbs up gesture
            1  # Queue size 1
        )
        
        # Declare parameters for performance / output control
        self.declare_parameter('processing_width', 640)
        self.declare_parameter('publish_annotated', True)
        self.declare_parameter('annotation_decimation', 1)
        self.processing_width = int(self.get_parameter('processing_width').value)
        self.publish_annotated = bool(self.get_parameter('publish_annotated').value)
        self.annotation_decimation = max(1, int(self.get_parameter('annotation_decimation').value))
        self.annotation_counter = 0

        # Publisher für annotiertes Bild (optional)
        if self.publish_annotated:
            self.image_pub = self.create_publisher(
                Image,
                '/hand/annotated_image',
                1  # Queue size 1
            )
        # Declare parameters to tune robustness without editing code
        self.declare_parameter('min_detection_frames', 1)
        self.declare_parameter('finger_extension_ratio', 0.18)
        self.declare_parameter('thumb_vertical_ratio', 0.12)
        self.declare_parameter('fold_margin_ratio', 0.04)

        self.min_detection_frames = max(1, int(self.get_parameter('min_detection_frames').value))
        self.finger_extension_ratio = float(self.get_parameter('finger_extension_ratio').value)
        self.thumb_vertical_ratio = float(self.get_parameter('thumb_vertical_ratio').value)
        self.fold_margin_ratio = float(self.get_parameter('fold_margin_ratio').value)

        # Simple temporal smoothing to reduce flicker between finger/thumb detections
        self.finger_detect_counter = 0
        self.thumb_detect_counter = 0
        self.finger_candidate = None
        self.thumb_candidate = None
        
        self.get_logger().info('👋 Hand Gesture Detector gestartet ✅')
        self.get_logger().info('Subscribed to /image')
        self.get_logger().info('Publishing to /hand/pointer_finger and /hand/thumbs_up')
    
    def estimate_hand_scale(self, landmarks):
        """Estimate hand scale based on wrist to middle finger MCP distance."""
        wrist = landmarks[self.mp_hands.HandLandmark.WRIST]
        middle_mcp = landmarks[self.mp_hands.HandLandmark.MIDDLE_FINGER_MCP]
        dx = wrist.x - middle_mcp.x
        dy = wrist.y - middle_mcp.y
        dz = wrist.z - middle_mcp.z
        return math.sqrt(dx * dx + dy * dy + dz * dz) + 1e-6

    def is_index_finger_up(self, landmarks, hand_scale):
        """
        Überprüft ob der Zeigefinger gehoben ist
        MediaPipe Landmark IDs:
        - WRIST: 0
        - THUMB_CMC, THUMB_MCP, THUMB_IP, THUMB_TIP: 1-4
        - INDEX_FINGER_MCP, INDEX_FINGER_PIP, INDEX_FINGER_DIP, INDEX_FINGER_TIP: 5-8
        """
        # Zeigefinger-Punkte
        index_tip = landmarks[self.mp_hands.HandLandmark.INDEX_FINGER_TIP]
        index_pip = landmarks[self.mp_hands.HandLandmark.INDEX_FINGER_PIP]
        index_mcp = landmarks[self.mp_hands.HandLandmark.INDEX_FINGER_MCP]

        extension_threshold = self.finger_extension_ratio * hand_scale
        # Überprüfe, dass Finger deutlich gestreckt ist (Tip deutlich über PIP & MCP)
        tip_above_pip = (index_pip.y - index_tip.y) > extension_threshold
        tip_above_mcp = (index_mcp.y - index_tip.y) > extension_threshold
        # Finger soll nahezu gerade sein -> Abstand zwischen Tip und PIP größer als zwischen PIP und MCP
        pip_to_tip = abs(index_pip.y - index_tip.y)
        mcp_to_pip = abs(index_mcp.y - index_pip.y)
        straight_enough = pip_to_tip > (0.7 * mcp_to_pip)
        
        is_up = tip_above_pip and tip_above_mcp and straight_enough
        return is_up, index_tip
    
    def is_thumbs_up(self, landmarks, hand_scale):
        """
        Überprüft ob der Daumen gehoben ist
        Thumbs Up Erkennung: Daumen ausgebreitet und nach oben
        """
        # Daumen-Punkte
        thumb_tip = landmarks[self.mp_hands.HandLandmark.THUMB_TIP]
        thumb_ip = landmarks[self.mp_hands.HandLandmark.THUMB_IP]
        thumb_mcp = landmarks[self.mp_hands.HandLandmark.THUMB_MCP]
        thumb_cmc = landmarks[self.mp_hands.HandLandmark.THUMB_CMC]
        
        vertical_threshold = self.thumb_vertical_ratio * hand_scale

        # Prüfe ob Daumen nach oben zeigt (y abnehmend) und weitgehend vertikal verläuft
        thumb_vec_y = thumb_ip.y - thumb_tip.y  # positiv wenn Tip deutlich über IP
        thumb_vec_x = abs(thumb_tip.x - thumb_ip.x)
        vertical_enough = thumb_vec_y > vertical_threshold and thumb_vec_y > thumb_vec_x
        thumb_up = vertical_enough and (thumb_tip.y < thumb_ip.y < thumb_mcp.y)
        
        # Alle anderen Finger sollten unten sein (closed fist oder gerade)
        index_tip = landmarks[self.mp_hands.HandLandmark.INDEX_FINGER_TIP]
        middle_tip = landmarks[self.mp_hands.HandLandmark.MIDDLE_FINGER_TIP]
        ring_tip = landmarks[self.mp_hands.HandLandmark.RING_FINGER_TIP]
        pinky_tip = landmarks[self.mp_hands.HandLandmark.PINKY_TIP]
        
        # Finger sollten niedriger als MCP sein (gefaltet)
        fold_margin = self.fold_margin_ratio * hand_scale
        fingers_down = (
            index_tip.y > landmarks[self.mp_hands.HandLandmark.INDEX_FINGER_MCP].y - fold_margin and
            middle_tip.y > landmarks[self.mp_hands.HandLandmark.MIDDLE_FINGER_MCP].y - fold_margin and
            ring_tip.y > landmarks[self.mp_hands.HandLandmark.RING_FINGER_MCP].y - fold_margin and
            pinky_tip.y > landmarks[self.mp_hands.HandLandmark.PINKY_MCP].y - fold_margin
        )
        
        is_thumbs_up = thumb_up and fingers_down
        
        return is_thumbs_up, thumb_tip
    
    def image_callback(self, msg):
        """Verarbeitet eingehende Bilder"""
        try:
            cv_image, processed_image, annotated_image = self._prepare_image(msg)
            results = self.hands.process(processed_image)

            (
                finger_detected,
                finger_position,
                thumbs_up_detected,
                thumbs_up_position,
                annotated_image,
            ) = self._analyze_landmarks(results, cv_image, annotated_image)

            self._publish_gesture_points(
                msg,
                finger_detected,
                finger_position,
                thumbs_up_detected,
                thumbs_up_position,
            )
            self._publish_annotated_image(msg, annotated_image)
        except Exception as e:
            self.get_logger().error(f'Fehler bei Bildverarbeitung: {e}')

    def _prepare_image(self, msg):
        """Convert ROS image to BGR, optionally downscale, and prepare RGB copy."""
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        orig_h, orig_w = cv_image.shape[:2]
        processed_image = cv_image
        if self.processing_width > 0 and orig_w > self.processing_width:
            scale = self.processing_width / float(orig_w)
            target_size = (self.processing_width, max(1, int(orig_h * scale)))
            processed_image = cv2.resize(cv_image, target_size, interpolation=cv2.INTER_LINEAR)
        processed_image = cv2.cvtColor(processed_image, cv2.COLOR_BGR2RGB)
        annotated_image = cv_image.copy()
        return cv_image, processed_image, annotated_image

    def _analyze_landmarks(self, results, cv_image, annotated_image):
        """Evaluate all detected hands and return gesture flags plus annotated frame."""
        finger_detected = False
        finger_position = None
        thumbs_up_detected = False
        thumbs_up_position = None

        if results.multi_hand_landmarks:
            h, w = cv_image.shape[:2]
            for hand_landmarks in results.multi_hand_landmarks:
                self._draw_hand(annotated_image, hand_landmarks)
                landmarks = hand_landmarks.landmark
                hand_scale = self.estimate_hand_scale(landmarks)
                detected, position = self._update_finger_detection(
                    landmarks, hand_scale, h, w, annotated_image
                )
                if detected:
                    finger_detected = True
                    finger_position = position
                detected, position = self._update_thumb_detection(
                    landmarks, hand_scale, h, w, annotated_image
                )
                if detected:
                    thumbs_up_detected = True
                    thumbs_up_position = position

        return finger_detected, finger_position, thumbs_up_detected, thumbs_up_position, annotated_image

    def _draw_hand(self, annotated_image, hand_landmarks):
        """Render a MediaPipe hand skeleton onto the annotated image."""
        self.mp_draw.draw_landmarks(
            annotated_image,
            hand_landmarks,
            self.mp_hands.HAND_CONNECTIONS,
            self.mp_draw.DrawingSpec(color=(0, 255, 0), thickness=2),
            self.mp_draw.DrawingSpec(color=(255, 0, 0), thickness=2)
        )

    def _update_finger_detection(self, landmarks, hand_scale, image_height, image_width, annotated_image):
        """Update index-finger detection counters and draw/return the point if stable."""
        is_up, index_tip = self.is_index_finger_up(landmarks, hand_scale)
        if is_up:
            self.finger_candidate = (
                int(index_tip.x * image_width),
                int(index_tip.y * image_height),
                index_tip.z,
            )
            self.finger_detect_counter += 1
        else:
            self.finger_detect_counter = 0
            self.finger_candidate = None
        if self.finger_detect_counter >= self.min_detection_frames and self.finger_candidate:
            finger_position = self.finger_candidate
            cv2.circle(
                annotated_image,
                (finger_position[0], finger_position[1]),
                10,
                (0, 255, 255),
                -1,
            )
            self.get_logger().info(
                f'👆 Zeigefinger erkannt: x={finger_position[0]}, y={finger_position[1]}'
            )
            return True, finger_position
        return False, None

    def _update_thumb_detection(self, landmarks, hand_scale, image_height, image_width, annotated_image):
        """Update thumbs-up detection counters and draw/return the point if stable."""
        thumbs_up, thumb_tip = self.is_thumbs_up(landmarks, hand_scale)
        if thumbs_up:
            self.thumb_candidate = (
                int(thumb_tip.x * image_width),
                int(thumb_tip.y * image_height),
                thumb_tip.z,
            )
            self.thumb_detect_counter += 1
        else:
            self.thumb_detect_counter = 0
            self.thumb_candidate = None
        if self.thumb_detect_counter >= self.min_detection_frames and self.thumb_candidate:
            thumb_position = self.thumb_candidate
            cv2.circle(
                annotated_image,
                (thumb_position[0], thumb_position[1]),
                15,
                (255, 255, 0),
                -1,
            )
            self.get_logger().info(
                f'👍 Thumbs Up erkannt: x={thumb_position[0]}, y={thumb_position[1]}'
            )
            return True, thumb_position
        return False, None

    def _publish_gesture_points(self, msg, finger_detected, finger_position, thumbs_up_detected, thumbs_up_position):
        """Publish PointStamped messages for the detected finger/thumb gestures."""
        if finger_detected and finger_position:
            point_msg = PointStamped()
            point_msg.header.stamp = msg.header.stamp
            point_msg.header.frame_id = msg.header.frame_id
            point_msg.point.x = float(finger_position[0])
            point_msg.point.y = float(finger_position[1])
            point_msg.point.z = float(finger_position[2])
            self.finger_pub.publish(point_msg)

        if thumbs_up_detected and thumbs_up_position:
            point_msg = PointStamped()
            point_msg.header.stamp = msg.header.stamp
            point_msg.header.frame_id = msg.header.frame_id
            point_msg.point.x = float(thumbs_up_position[0])
            point_msg.point.y = float(thumbs_up_position[1])
            point_msg.point.z = float(thumbs_up_position[2])
            self.thumbs_up_pub.publish(point_msg)

    def _publish_annotated_image(self, msg, annotated_image):
        """Publish the annotated debug image according to the decimation setting."""
        if not self.publish_annotated:
            return
        self.annotation_counter = (self.annotation_counter + 1) % self.annotation_decimation
        if self.annotation_counter == 0:
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated_image, 'bgr8')
            annotated_msg.header = msg.header
            self.image_pub.publish(annotated_msg)

def main(args=None):
    rclpy.init(args=args)
    node = HandGestureDetector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Hand Gesture Detector beendet 🛑')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

