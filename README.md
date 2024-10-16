# Gesture Classifier

This repository contains the ROS2 package `gesture_classifier`, which uses MediaPipe to recognize hand gestures from a video stream and publishes annotated images.

The classes are:
- 0 - Non, label: **Unknown**
- 1 - ✊, label: **Closed_Fist**
- 2 - 👋, label: **Open_Palm**
- 3 - ☝️, label: **Pointing_Up**
- 4 - 👎, label: **Thumb_Down**
- 5 - 👍, label: **Thumb_Up**
- 6 - ✌️, label: **Victory**
- 7 - 🤟, label: **ILoveYou**


For more info:

- [Google AI for Developers guide for gesture recognition](https://ai.google.dev/edge/mediapipe/solutions/vision/gesture_recognizer?hl=pt-br#models)

- [MediaPipe AI Model Card](https://storage.googleapis.com/mediapipe-assets/gesture_recognizer/model_card_hand_gesture_classification_with_faireness_2022.pdf)



## Prerequisites

Before you begin, ensure you have met the following requirements:

- **Operating System:** Ubuntu 22.04 for Raspberry Pi
- **ROS2:** ROS2 Humble
- **Python:** Python 3.8 or later
- **Camera:** Raspberry Pi Camera Module (e.g., RaspCam V1.3) or computer webcam

### Install ROS2

Follow the official ROS2 installation guide for [Ubuntu](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) or [Raspberry Pi OS](https://docs.ros.org/en/humble/Installation/Raspberry-Pi-Install.html).

### Install Additional Dependencies

1. **Install `cv_bridge` and other ROS2 dependencies:**

   ```sh
   sudo apt-get update
   sudo apt-get install ros-humble-cv-bridge ros-humble-image-transport ros-humble-compressed-image-transport
   ```

2. **Install MediaPipe and OpenCV for Python:**

   ```sh
   pip install mediapipe opencv-python
   ```

## Installation

1. **Clone the Repository:**

   ```sh
   git clone https://github.com/Equipe-eVTOL-ITA/gesture_classifier.git
   cd gesture_classifier
   ```

2. **Build the Package:**

   ```sh
   colcon build
   source install/setup.bash
   ```

## Usage

1. **Run the Gesture Classifier Node:**

   Note: remember to `source /opt/ros/humble/setup.bash` and `source install/setup.bash`

   ```sh
   ros2 run gesture_classifier gesture_classifier --ros-args -p num_hands:=2
   ```

2. **View the Annotated Images:**

   You can use `rqt_image_view` to view the annotated images:

   ```sh
   rqt_image_view /camera/image_annotated/compressed
   ```

   Or when on computer:

   ```sh
   source /opt/ros/humble/setup.bash
   ros2 run rqt_image_view rqt_image_view
   ```

## Explanation

The `gesture_classifier` package captures images , processes them using MediaPipe to recognize hand gestures, and publishes annotated images on the `/camera/gesture/compressed` topic. This package is useful for applications requiring hand gesture recognition and control.


## References

- [ROS2 Documentation](https://docs.ros.org/en/humble/index.html)
- [OpenCV Documentation](https://docs.opencv.org/master/)
- [MediaPipe Documentation](https://google.github.io/mediapipe/)
- [Raspberry Pi Camera Module](https://www.raspberrypi.org/documentation/accessories/camera.md)
