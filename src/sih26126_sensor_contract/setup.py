from setuptools import find_packages, setup

package_name = 'sih26126_sensor_contract'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        (
            'share/' + package_name,
            ['package.xml'],
        ),
        (
            'share/' + package_name + '/config',
            ['config/topics.yaml'],
        ),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='venom',
    maintainer_email='venom@todo.todo',
    description=(
        'Sensor health, freshness, depth quality, and '
        'synchronization monitoring for DRISTHI.'
    ),
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_contract = '
            'sih26126_sensor_contract.sensor_contract:main',
            'lidar_monitor = '
            'sih26126_sensor_contract.lidar_monitor:main',
            'scan_relay = '
            'sih26126_sensor_contract.scan_relay:main',
            'lidar_processor = '
            'sih26126_sensor_contract.lidar_processor:main',
            'obstacle_extractor = '
            'sih26126_sensor_contract.obstacle_extractor:main',
            'obstacle_tracker = '
            'sih26126_sensor_contract.obstacle_tracker:main',
        ],
    },
)
