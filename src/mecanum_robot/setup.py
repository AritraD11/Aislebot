import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'mecanum_robot'

setup(
    name=package_name,
    version='3.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (os.path.join('share', package_name, 'resource'), ['resource/dashboard.html']),
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Aritra Das',
    maintainer_email='aritra@iitb.ac.in',
    description='AisleBot asymmetric mecanum robot — control, teleop, simulation',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'esp32_bridge = mecanum_robot.esp32_bridge:main',
            'teleop_asym = mecanum_robot.mecanum_teleop_asymmetric:main',
            'odom_pub = mecanum_robot.odometry_publisher:main',
            'keyboard_teleop = mecanum_robot.keyboard_teleop:main',
            'gazebo_bridge = mecanum_robot.gazebo_bridge:main',
            'arm_bridge        = mecanum_robot.arm_bridge:main',
            'joy_to_aislebot   = mecanum_robot.joy_to_aislebot:main',
            'phone_dashboard   = mecanum_robot.phone_dashboard:main',
            'lcd_display       = mecanum_robot.lcd_display:main',
        ],
    },
)
