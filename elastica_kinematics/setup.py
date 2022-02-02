from setuptools import setup
import os
from glob import glob

package_name = 'elastica_kinematics'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=['setuptools', 'pyelastica', 'matplotlib', 'numpy', 'moviepy', 'ffmpeg'],
    zip_safe=True,
    author='Prakarsh Kaushik',
    author_email='prakarshkaushik369@gmail.com',
    maintainer='Prakarsh Kaushik',
    maintainer_email='prakarshkaushik369@gmail.com',
    description='ROS2 package for Kinematics of Cosserat Rods assmebled in PyElastica',
    license='MIT license',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'elastica_rod_states_to_PCC = elastica_kinematics.elastica_rod_states_to_PCC:main',
            'elastica_forward_kinematics = elastica_kinematics.elastica_forward_kinematics:main',
        ],
    },
)
