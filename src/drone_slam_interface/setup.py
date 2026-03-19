from setuptools import setup

package_name = 'drone_slam_interface'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'slam_bridge = drone_slam_interface.slam_bridge:main',
            'demo_visualizer = drone_slam_interface.demo_visualizer:main',
        ],
    },
)