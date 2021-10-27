from setuptools import setup

package_name = 'ros2_elastica_pack'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = ros2_elastica_pack.publisher_member_function:main',
            'continuum_flagella_ros2 = ros2_elastica_pack.continuum_flagella_ros2:main',
            'continuum_snake_ros2 = ros2_elastica_pack.continuum_snake_ros2:main',
        ],
},
)
