from setuptools import setup

package_name = 'polygon_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, 'polygon_nav.vosk_mic_listener'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'shapely', 'sounddevice', 'numpy', 'pyaudio', 'mediapipe', 'opencv-python'],
    zip_safe=True,
    maintainer='Dein Name',
    maintainer_email='dein@email.de',
    description='Polygon navigation client using nav2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'yolo_processor = polygon_nav.yolo_processor:main',
        'hand_gesture_detector = polygon_nav.hand_gesture_detector:main',
        'sensor_fusion = polygon_nav.sensor_fusion:main',
        'state_machine = polygon_nav.state_machine:main',
        'follower_control = polygon_nav.follower_control:main',
        'vosk_voice_assistant = polygon_nav.vosk_mic_listener.mic_node:main',
        'pose_control = polygon_nav.pose_control:main',

    ],
},
)


