from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'robot_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
        os.path.join('share', package_name, 'launch'),
        glob('launch/*.launch.py')
        ),

        (
        os.path.join('share', package_name, 'rviz'),
        glob('rviz/*.rviz')
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rishi',
    maintainer_email='rishi@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "ownmap = robot_navigation.mapping_by_grid:main",
            'navigator = robot_navigation.navigation_server:main'
        ],
    },
)
