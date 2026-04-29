from setuptools import setup
import os
from glob import glob

package_name = 'ida_otonom'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'missions'), glob('ida_otonom/missions/*.json')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='team',
    maintainer_email='team@example.com',
    description='TEKNOFEST IDA autonomy package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_manager_node = ida_otonom.mission_manager_node:main',
            'gps_guidance_node = ida_otonom.gps_guidance_node:main',
            'controller_node = ida_otonom.controller_node:main',
            'mavros_bridge_node = ida_otonom.mavros_bridge_node:main',
            'perception_node = ida_otonom.perception_node:main',
            'logger_node = ida_otonom.logger_node:main',
            'yki_bridge_node = ida_otonom.yki_bridge_node:main',
            'safety_node = ida_otonom.safety_node:main',
        ],
    },
)