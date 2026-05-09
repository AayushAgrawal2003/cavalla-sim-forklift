from setuptools import setup

package_name = 'mujoco_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='MuJoCo physics bridge for Cavalier forklift control stack.',
    license='TODO',
    entry_points={
        'console_scripts': [
            'mujoco_driver = mujoco_bridge.mujoco_driver_node:main',
            'automation_cmd_adapter = mujoco_bridge.automation_cmd_adapter:main',
        ],
    },
)
