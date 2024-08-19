import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import os
import cv2
import time
from .gesture_recognizer import GestureRecognizer
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import argparse

class GestureClassifier(Node):
    def __init__(self, num_hands):
        super().__init__('gesture_classifier')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        self.subscription = self.create_subscription(
            CompressedImage,
            'camera/image/compressed',
            self.listener_callback,
            qos_profile)
        self.publisher_annotated = self.create_publisher(
            CompressedImage, 'camera/gesture/compressed', qos_profile)
        self.publisher_gesture = self.create_publisher(
            String, 'gesture/classification', qos_profile)
        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()

        # Initialize the gesture recognizer with the specified number of hands
        script_dir = os.path.dirname(os.path.abspath(__file__))
        model_path = os.path.join(script_dir, 'gesture_recognizer.task')
        self.recognizer = GestureRecognizer(
            model=model_path,
            num_hands=num_hands,
            min_hand_detection_confidence=0.5,
            min_hand_presence_confidence=0.5,
            min_tracking_confidence=0.5
        )
        self.get_logger().info(f'GestureClassifier node started with num_hands={num_hands}')

        # Throttle classification frequency
        self.last_classification_time = time.time()
        self.classification_interval = 0.1  # seconds - Increased interval for throttling

        # Initialize last recognized gestures and landmarks
        self.last_gestures = [None, None]
        self.last_hand_landmarks = []

    def listener_callback(self, msg):
        current_time = time.time()

        if current_time - self.last_classification_time < self.classification_interval:
            # Skip processing if within throttle interval
            return

        try:
            frame = self.bridge.compressed_imgmsg_to_cv2(msg)
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')
            return

        result = self.recognizer.recognize(frame)
        self.last_classification_time = current_time

        if result:
            #self.get_logger().info(f"Result gestures: {result.gestures}")

            # Update the last recognized gestures and landmarks
            if result.gestures:
                self.last_gestures = [gesture[0] if gesture else None for gesture in result.gestures[:2]]
            else:
                self.last_gestures = [None, None]

            if result.hand_landmarks:
                self.last_hand_landmarks = result.hand_landmarks[:2]
            else:
                self.last_hand_landmarks = []

            # Publish the gesture classification as a list of strings
            gestures = []
            if len(self.last_gestures) > 0 and self.last_gestures[0] is not None:
                gestures.append(self.last_gestures[0].category_name)
            if len(self.last_gestures) > 1 and self.last_gestures[1] is not None:
                gestures.append(self.last_gestures[1].category_name)

            for gesture in gestures:
                gesture_msg = String()
                gesture_msg.data = gesture
                self.publisher_gesture.publish(gesture_msg)
        else:
            self.last_gestures = [None, None]
            self.last_hand_landmarks = []

        # Annotate the frame with last recognized landmarks and gestures
        for i, hand_landmarks in enumerate(self.last_hand_landmarks):
            for landmark in hand_landmarks:
                cx, cy = int(landmark.x * frame.shape[1]), int(landmark.y * frame.shape[0])
                cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)

            if len(self.last_gestures) > i and self.last_gestures[i] is not None:
                category_name = self.last_gestures[i].category_name
                score = round(self.last_gestures[i].score, 2)
                self.get_logger().info(f'Gesture {i+1}: {category_name} ({score})')

                # Annotate the frame with the gesture
                result_text = f'{category_name} ({score})'
                x_min_px = int(min([landmark.x for landmark in hand_landmarks]) * frame.shape[1])
                y_min_px = int(min([landmark.y for landmark in hand_landmarks]) * frame.shape[0])
                y_max_px = int(max([landmark.y for landmark in hand_landmarks]) * frame.shape[0])
                text_size = cv2.getTextSize(result_text, cv2.FONT_HERSHEY_DUPLEX, 1, 2)[0]
                text_x = x_min_px
                text_y = y_min_px - 10 if y_min_px - 10 > 10 else y_max_px + text_size[1]

                cv2.putText(frame, result_text, (text_x, text_y), cv2.FONT_HERSHEY_DUPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)

        # Publish the annotated image
        try:
            annotated_image_msg = self.bridge.cv2_to_compressed_imgmsg(frame)
            annotated_image_msg.header.stamp = self.get_clock().now().to_msg()
            annotated_image_msg.header.frame_id = 'camera_frame'
            self.publisher_annotated.publish(annotated_image_msg)
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')

    def destroy_node(self):
        self.recognizer.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)

    # Argument parser to get the number of hands
    parser = argparse.ArgumentParser(description='Gesture Classifier Node')
    parser.add_argument('--num_hands', type=int, default=1, help='Number of hands to detect (1 or 2)')
    args = parser.parse_args()

    gesture_classifier = GestureClassifier(num_hands=args.num_hands)
    rclpy.spin(gesture_classifier)
    gesture_classifier.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
