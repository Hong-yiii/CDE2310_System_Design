from setuptools import setup, find_packages

package_name = 'turtlebot3_flywheels_and_temp'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),  # ✅ Automatically finds your inner Python package
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/bringup.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Example bringup for flywheels and AMG8833 temperature sensor',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'amg8833_publisher = turtlebot3_flywheels_and_temp.amg8833_publisher:main',
            'flywheel_node = turtlebot3_flywheels_and_temp.flywheel_node:main',
        ],
    },
)
