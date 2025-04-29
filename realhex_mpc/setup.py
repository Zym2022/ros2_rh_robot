from setuptools import setup
import os
from glob import glob

package_name = 'realhex_mpc'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Zhuyiming',
    maintainer_email='your-email@example.com',
    description='MPC接口用于连接OCS2 MPC控制器与RealHex机器人的Gazebo仿真',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'realhex_mpc_interface = realhex_mpc.realhex_mpc_interface:main',
            'realhex_mpc_state_publisher = realhex_mpc.realhex_mpc_state_publisher:main',
        ],
    },
) 