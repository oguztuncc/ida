from setuptools import find_packages, setup

package_name = 'ida_otonom'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='talay',
    maintainer_email='talay@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'lidar_node = ida_otonom.lidar_node:main',
            'yolo_node = ida_otonom.yolo_node:main',
            'gps_node = ida_otonom.gps_node:main',
            'beyin_node = ida_otonom.beyin_node:main',
        ],
    },
)
