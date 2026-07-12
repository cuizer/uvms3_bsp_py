import os
from glob import glob

from setuptools import setup

package_name = 'bsp'

setup(
    name=package_name,
    version='0.0.1',
    py_modules=[
        'teleop_receiver',
        'bsp_motioncontrol_node',
        'dof_controller',
        'thrust_alloc',
    ],
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py'),
        ),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='UVMS 3rd Gen Board Support Package (BSP) — Python layer',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_receiver = teleop_receiver:main',
            'bsp_motioncontrol_node = bsp_motioncontrol_node:main',
        ],
    },
)
