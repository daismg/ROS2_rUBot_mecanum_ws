from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'my_robot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='manel.puig@ub.edu',
    description='TODO: Package description',
    license='Apache License 2.0', # improved license
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_robot_lidar_test_exec = my_robot_control.my_robot_lidar_test:main',
            'my_robot_lidar_test_LIMO_exec = my_robot_control.my_robot_lidar_test_LIMO:main',
            'my_robot_control_exec = my_robot_control.my_robot_control:main',
            'my_robot_control_grup4_exec = my_robot_control.my_robot_control_grup4:main',
            'my_robot_selfcontrol_exec = my_robot_control.my_robot_selfcontrol:main',
            'my_robot_selfcontrol_LIMO_exec = my_robot_control.my_robot_selfcontrol_LIMO:main',
            'my_robot_wallfollower_exec = my_robot_control.my_robot_wallfollower:main',
            'my_robot_wallfollower_LIMO_exec = my_robot_control.my_robot_wallfollower_LIMO:main',
            'my_robot_go2pose_exec = my_robot_control.my_robot_go2pose:main',
        ],
    },
)
