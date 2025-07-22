import os
from glob import glob
from setuptools import setup

package_name = 'rh_moveit'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config', 'mobile_base'), glob('config/mobile_base/*')),
        (os.path.join('share', package_name, 'config', 'realhex'), glob('config/realhex/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='qiufangzhou',
    maintainer_email='373346856@qq.com',
    description='MoveIt configuration for the RealHex robot.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
) 