from setuptools import find_packages, setup

package_name = 'sih26126_autonomy'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='venom',
    maintainer_email='venom@todo.todo',
    description='Online occupancy mapping and frontier exploration.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'odometry_pose_adapter = '
            'sih26126_autonomy.online_mapping:pose_main',
            'online_mapper = '
            'sih26126_autonomy.online_mapping:mapper_main',
            'frontier_explorer = '
            'sih26126_autonomy.frontier_explorer:main',
        ],
    },
)
