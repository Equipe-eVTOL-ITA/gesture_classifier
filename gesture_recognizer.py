# gesture_recognizer.py

import time
import cv2
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from mediapipe.framework.formats import landmark_pb2

# Global variables to calculate FPS
COUNTER, FPS = 0, 0
START_TIME = time.time()

mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles

class GestureRecognizer:
    def __init__(self, model: str, num_hands: int,
                 min_hand_detection_confidence: float,
                 min_hand_presence_confidence: float,
                 min_tracking_confidence: float):
        self.recognition_frame = None
        self.recognition_result_list = []

        # Initialize the gesture recognizer model
        base_options = python.BaseOptions(model_asset_path=model)
        options = vision.GestureRecognizerOptions(base_options=base_options,
                                                  running_mode=vision.RunningMode.LIVE_STREAM,
                                                  num_hands=num_hands,
                                                  min_hand_detection_confidence=min_hand_detection_confidence,
                                                  min_hand_presence_confidence=min_hand_presence_confidence,
                                                  min_tracking_confidence=min_tracking_confidence,
                                                  result_callback=self.save_result)
        self.recognizer = vision.GestureRecognizer.create_from_options(options)

    def save_result(self, result: vision.GestureRecognizerResult, unused_output_image: mp.Image, timestamp_ms: int):
        global FPS, COUNTER, START_TIME

        # Calculate the FPS
        if COUNTER % 10 == 0:  # fps_avg_frame_count = 10
            FPS = 10 / (time.time() - START_TIME)
            START_TIME = time.time()

        self.recognition_result_list.append(result)
        COUNTER += 1

    def recognize(self, frame):
        rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_image)
        self.recognizer.recognize_async(mp_image, time.time_ns() // 1_000_000)

        if self.recognition_result_list:
            results = self.recognition_result_list[0]
            self.recognition_result_list.clear()
            return results
        return None

    def close(self):
        self.recognizer.close()
