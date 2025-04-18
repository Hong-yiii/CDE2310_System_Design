from setuptools import setup, find_packages

package_name = 'bot_bestie'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/launch', [
            f'{package_name}/launch/global_bringup.py',
            f'{package_name}/launch/global_controller_bringup.py',
            f'{package_name}/launch/nav2_bringup.py'
        ]),
        (f'share/{package_name}/param', [
            f'{package_name}/param/burger.yaml'   # ← ✅ Add this line
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hongyi',
    maintainer_email='hongyilin.mail@gmail.com',
    description='ROS 2 bot_bestie navigation package',
    license='MIT',
    entry_points={
        'console_scripts': [
            'global_controller = bot_bestie.nodes.global_controller:main',
            'navigation_node = bot_bestie.nodes.navigation_node:main',
        ],
    },
)
