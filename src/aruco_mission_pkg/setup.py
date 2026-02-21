from setuptools import setup

package_name = 'aruco_mission_pkg'

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
    maintainer='ricardo',
    maintainer_email='ricardo@todo.todo',
    description='PX4 Aruco Mission Package',
    license='TODO',
    entry_points={
        'console_scripts': [
            'aruco_detector = aruco_mission_pkg.aruco_detector_node:main',
            'mission_sm = aruco_mission_pkg.state_machine:main',
        ],
    },
)