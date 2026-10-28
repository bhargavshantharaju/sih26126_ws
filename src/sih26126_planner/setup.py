from setuptools import find_packages, setup

package_name = 'sih26126_planner'

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
    description='Terrain-aware advisory planner baseline for DRISTHI.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'goal_path_planner = '
            'sih26126_planner.goal_path_planner:main',
            'local_path_selector = '
            'sih26126_planner.local_path_selector:main',
            'planner_baseline = '
            'sih26126_planner.planner_baseline:main',
            'cost_map_fusion = '
            'sih26126_planner.cost_map_fusion:main',
        ],
    },
)
