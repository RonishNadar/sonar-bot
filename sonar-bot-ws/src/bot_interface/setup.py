from setuptools import setup

package_name = 'bot_interface'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ronish Nadar',
    maintainer_email='nadar.ronish@email.com',
    description='ROS2 interface for Sonar ESP32 robot',
    license='MIT',
    entry_points={
        'console_scripts': [
            'bot_node = bot_interface.bot_node:main',
            'lidar_visualizer = bot_interface.lidar_visualizer:main',
        ],
    },
)
