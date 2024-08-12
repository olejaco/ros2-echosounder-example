from setuptools import setup

package_name = 'echo_sounder_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='ROS 2 Echo-Sounder System',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_node = echo_sounder_pkg.sensor_node:main',
            'processor_node = echo_sounder_pkg.processor_node:main',
            'visualizer_node = echo_sounder_pkg.visualizer_node:main',
        ],
    },
)