import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import cv2
import time
from gesture_recognizer import GestureRecognizer
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class GestureClassifier(Node):
    def __init__(self):
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
            CompressedImage, 'camera/image_annotated/compressed', qos_profile)
        self.publisher_gesture = self.create_publisher(
            String, 'gesture/classification', qos_profile)
        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()

        # Initialize the gesture recognizer
        model_path = '/home/evtol/ws_sensor_combined/src/gesture_classifier/gesture_classifier/gesture_recognizer.task'
        self.recognizer = GestureRecognizer(
            model=model_path,
            num_hands=1,
            min_hand_detection_confidence=0.5,
            min_hand_presence_confidence=0.5,
            min_tracking_confidence=0.5
        )
        self.get_logger().info('GestureClassifier node started')

        # Throttle classification frequency
        self.last_classification_time = time.time()
        self.classification_interval = 0.1  # seconds

        # Initialize last recognized gesture and landmarks
        self.last_gesture = None
        self.last_hand_landmarks = []

    def listener_callback(self, msg):
        try:
            frame = self.bridge.compressed_imgmsg_to_cv2(msg)
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')
            return

        # Check if it's time to classify
        current_time = time.time()
        if current_time - self.last_classification_time >= self.classification_interval:
            result = self.recognizer.recognize(frame)
            self.last_classification_time = current_time

            # Update the last recognized gesture and landmarks
            if result:
                self.last_gesture = result.gestures[0] if result.gestures else None
                self.last_hand_landmarks = result.hand_landmarks if result.hand_landmarks else []

                # Publish the gesture class
                if self.last_gesture:
                    gesture_class = self.last_gesture[0].category_name
                    gesture_msg = String()
                    gesture_msg.data = gesture_class
                    self.publisher_gesture.publish(gesture_msg)
        else:
            result = None

        # Annotate the frame with last recognized landmarks and gesture
        for hand_landmarks in self.last_hand_landmarks:
            for landmark in hand_landmarks:
                cx, cy = int(landmark.x * frame.shape[1]), int(landmark.y * frame.shape[0])
                cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)

        if self.last_gesture:
            category_name = self.last_gesture[0].category_name
            score = round(self.last_gesture[0].score, 2)
            self.get_logger().info(f'Gesture: {category_name} ({score})')

            # Annotate the frame with the gesture
            result_text = f'{category_name} ({score})'
            x_min_px = int(min([landmark.x for landmark in self.last_hand_landmarks[0]]) * frame.shape[1])
            y_min_px = int(min([landmark.y for landmark in self.last_hand_landmarks[0]]) * frame.shape[0])
            y_max_px = int(max([landmark.y for landmark in self.last_hand_landmarks[0]]) * frame.shape[0])
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
    gesture_classifier = GestureClassifier()
    rclpy.spin(gesture_classifier)
    gesture_classifier.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

