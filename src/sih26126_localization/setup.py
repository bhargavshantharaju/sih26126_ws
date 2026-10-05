from setuptools import find_packages, setup

package_name = 'sih26126_localization'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', [
            'config/slam_params.yaml',
            'config/ekf_params.yaml',
        ]),
    ],
    install_requires=['setuptools', 'numpy', 'opencv-python'],
    zip_safe=True,
    maintainer='venom',
    maintainer_email='venom@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'rgbd_odometry_baseline = '
            'sih26126_localization.rgbd_odometry_baseline:main',
            'slam_pose_adapter = '
            'sih26126_localization.slam_pose_adapter:main',
            'odom_tf_publisher = '
            'sih26126_localization.odom_tf_publisher:main',
            'lidar_static_tf_publisher = '
            'sih26126_localization.lidar_static_tf_publisher:main',
            'imu_static_tf_publisher = '
            'sih26126_localization.imu_static_tf_publisher:main',
            'slam_activator = '
            'sih26126_localization.slam_activator:main',
            'slam_validation_test = '
            'sih26126_localization.slam_validation_test:main',
            'fusion_health_monitor = '
            'sih26126_localization.fusion_health_monitor:main',
            'ekf_sensor_adapter = '
            'sih26126_localization.ekf_sensor_adapter:main',
            'wheel_contribution_test = '
            'sih26126_localization.wheel_contribution_test:main',
        ],
    },
)
