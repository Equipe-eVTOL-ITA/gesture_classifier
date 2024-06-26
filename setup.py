from setuptools import setup, find_packages

package_name = 'gesture_classifier'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=['gesture_classifier', 'gesture_classifier.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/gesture_classifier_launch.py']),
        ('share/' + package_name + '/gesture_classiifer', ['gesture_classifier/gesture_recognizer.task']),
        ('share/' + package_name + '/gesture_classiifer', ['gesture_classifier/gesture_recognizer.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Package to classify hand gestures using MediaPipe.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gesture_classifier = gesture_classifier.gesture_classifier:main',
        ],
    },
)
