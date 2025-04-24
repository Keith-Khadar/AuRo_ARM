from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'maxarm'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='embedded',
    maintainer_email='keith.khadar@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arm_pos_pub = maxarm.arm_pos_publisher:main',
            'arm_set_pos = maxarm.arm_set_pos:main'
        ],
    },
)
