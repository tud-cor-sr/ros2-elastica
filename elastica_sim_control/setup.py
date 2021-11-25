from setuptools import setup

package_name = 'elastica_sim_control'

setup(
    name=package_name,
    version='0.1.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyelastica', 'matplotlib', 'numpy', 'moviepy', 'ffmpeg'],
    zip_safe=True,
    author='Prakarsh Kaushik',
    author_email='prakarshkaushik369@gmail.com',
    maintainer='Prakarsh Kaushik',
    maintainer_email='prakarshkaushik369@gmail.com',
    description='ROS2 package for PyElastica simulator for simulating assemblies of Cosserat Rods',
    license='MIT license',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'continuum_flagella_ros2 = elastica_sim_control.continuum_flagella_ros2:main',
            'continuum_snake_ros2 = elastica_sim_control.continuum_snake_ros2:main',
        ],
},
)
