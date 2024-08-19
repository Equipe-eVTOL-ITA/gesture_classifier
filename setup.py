from setuptools import setup

package_name = 'gesture_classifier'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Package to classify hand gestures using MediaPipe.',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'gesture_classifier = gesture_classifier.gesture_classifier:main',
        ],
    },
)
