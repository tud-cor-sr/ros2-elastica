from setuptools import setup
import os
from glob import glob

package_name = 'elastica_controllers'

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
    description='ROS2 package for PyElastica simulator for controlling assemblies of Cosserat Rods',
    license='MIT license',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'elastica_random_controller = elastica_controllers.elastica_random_controller:main',
        ],
    },
)
