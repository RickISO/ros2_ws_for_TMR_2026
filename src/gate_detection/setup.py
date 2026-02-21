from setuptools import setup

package_name = 'gate_detection_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=['gate_detection_pkg'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ricardo',
    maintainer_email='ricardo@todo.todo',
    description='Gate Detection PX4 Package',
    license='TODO',
    entry_points={
        'console_scripts': [
            'gate_detection = gate_detection_pkg.gate_detection_node:main',
            'mission_sm = gate_detection_pkg.state_machine:main',
        ],
    },
)