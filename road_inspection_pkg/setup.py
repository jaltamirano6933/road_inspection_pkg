from setuptools import setup

package_name = 'road_inspection_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/road_inspection.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Julio Altamirano',
    maintainer_email='',
    description='TurtleBot3 Road Inspection with ROS2 + OpenCV',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'road_inspection_node = road_inspection_pkg.road_inspection_node:main',
        ],
    },
)
