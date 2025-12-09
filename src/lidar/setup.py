from setuptools import find_packages, setup
from glob import glob
import os
package_name = 'lidar'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
         (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/**')),
        (os.path.join('share', package_name, 'config'), glob('config/**')),
        (os.path.join('share', package_name, 'world'), glob('world/**')),
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 必须手动把所有 .msg 文件列出来
        (os.path.join('share', package_name, 'msg'), glob('msg/*.msg')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xiluo',
    maintainer_email='wangzinuo233@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'rotate_wheel = lidar.rotate_wheel:main',
            'vel_to_joint =  lidar.cmd_vel_to_joint_state:main',
            'odom = lidar.odom_sim:main',
            'serial_read = lidar.serial_read:main',
            'odometry_pub = lidar.odometry_pub:main',
            'operator = lidar.operator:main',
        ],
    },
)
