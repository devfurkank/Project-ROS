from glob import glob

from setuptools import find_packages, setup

package_name = 'otonom_surus'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/worlds', glob('worlds/*.world')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Furkan Keskin',
    maintainer_email='keskin134@gmail.com',
    description='ROS 2 ile otonom sürüş simülasyonu: LiDAR engel algılama, '
                'SLAM, Nav2 ile planlama/kontrol ve görev yönetimi.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'obstacle_detector = otonom_surus.obstacle_detector:main',
            'mission_manager = otonom_surus.mission_manager:main',
            'obstacle_mover = otonom_surus.obstacle_mover:main',
        ],
    },
)
