import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'mujoco_pendulum'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install the MuJoCo model XML
        (os.path.join('share', package_name, 'models'),
            glob('models/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jiang',
    maintainer_email='jiang@todo.todo',
    description='MuJoCo inverted pendulum with LQR control via ROS 2',
    license='MIT',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'mj_lqr_node = mujoco_pendulum.mj_lqr_node:main',
            'mj_visualized_node = mujoco_pendulum.mj_visualized_node:main',
        ],
    },
)
