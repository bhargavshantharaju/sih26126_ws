from setuptools import find_packages, setup

package_name = 'sih26126_safety'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='venom',
    maintainer_email='venom@todo.todo',
    description='Safety supervisor for UGV command arbitration',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'safety_supervisor = '
            'sih26126_safety.safety_supervisor:main',
            'stuck_recovery = '
            'sih26126_safety.stuck_recovery:main',
            'degraded_autonomy = sih26126_safety.degraded_autonomy:main',
        ],
    },
)
