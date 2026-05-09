from glob import glob
import os

from setuptools import setup

package_name = 'drop_off'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cavallatestbench',
    maintainer_email='user@todo.todo',
    description='Drop-off line follower (yellow guide + stop line).',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'line_detector = drop_off.line_detector:main',
            'line_follower_controller = '
            'drop_off.line_follower_controller:main',
            'aruco_distance_guard = '
            'drop_off.aruco_distance_guard:main',
        ],
    },
)
