import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from gesture_recognizer import GestureRecognizer
import time

class GestureClassifier(Node):
    def __init__(self):
        super().__init__('gesture_classifier')
        self.subscription = self.create_subscription(
            CompressedImage,
            'camera/image/compressed',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(CompressedImage, 'camera/image_annotated/compressed', 10)
        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()

        # Initialize the gesture recognizer
        model_path = '/home/evtol/ws_sensor_combined/src/gesture_classifier/gesture_recognizer.task'
        self.recognizer = GestureRecognizer(
            model=model_path,
            num_hands=1,
            min_hand_detection_confidence=0.5,
            min_hand_presence_confidence=0.5,
            min_tracking_confidence=0.5
        )
        self.gesture_names = ['Gesture1', 'Gesture2', 'Gesture3']  # Modify according to your model
        self.get_logger().info('GestureClassifier node started')

        # Throttle classification frequency
        self.last_classification_time = time.time()
        self.classification_interval = 0.1 # seconds

    def listener_callback(self, msg):
        #self.get_logger().info('Image received')
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
        else:
            result = None

        # Annotate the frame with landmarks and gesture if recognized
        if result:
            for hand_index, hand_landmarks in enumerate(result.hand_landmarks):
                if result.gestures:
                    gesture = result.gestures[hand_index]
                    category_name = gesture[0].category_name
                    score = round(gesture[0].score, 2)
                    self.get_logger().info(f'Gesture: {category_name} ({score})')

                    # Annotate the frame with landmarks and gesture
                    x_min = min([landmark.x for landmark in hand_landmarks])
                    y_min = min([landmark.y for landmark in hand_landmarks])
                    y_max = max([landmark.y for landmark in hand_landmarks])

                    # Convert normalized coordinates to pixel values
                    frame_height, frame_width = frame.shape[:2]
                    x_min_px = int(x_min * frame_width)
                    y_min_px = int(y_min * frame_height)
                    y_max_px = int(y_max * frame_height)

                    result_text = f'{category_name} ({score})'
                    text_size = cv2.getTextSize(result_text, cv2.FONT_HERSHEY_DUPLEX, 1, 2)[0]
                    text_x = x_min_px
                    text_y = y_min_px - 10 if y_min_px - 10 > 10 else y_max_px + text_size[1]

                    cv2.putText(frame, result_text, (text_x, text_y), cv2.FONT_HERSHEY_DUPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
                    for landmark in hand_landmarks:
                        cx, cy = int(landmark.x * frame_width), int(landmark.y * frame_height)
                        cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)

        # Publish the annotated image (or original if no gesture recognized)
        try:
            annotated_image_msg = self.bridge.cv2_to_compressed_imgmsg(frame)
            annotated_image_msg.header.stamp = self.get_clock().now().to_msg()
            annotated_image_msg.header.frame_id = 'camera_frame'
            self.publisher_.publish(annotated_image_msg)
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
