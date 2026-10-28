from setuptools import find_packages, setup

package_name = 'sih26126_traversability'

setup(
    name=package_name,
    version='0.1.0',
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
    ],
    install_requires=[
        'setuptools',
    ],
    extras_require={
        'test': [
            'pytest',
        ],
    },
    zip_safe=True,
    maintainer='venom',
    maintainer_email='venom@todo.todo',
    description='Camera-based traversability estimation for DRISTHI.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'traversability_baseline = '
            'sih26126_traversability.traversability_baseline:main',
            'terrain_aware_traversability = '
            'sih26126_traversability.terrain_aware_traversability:main',
            'terrain_memory = '
            'sih26126_traversability.terrain_memory:main',
        ],
    },
)
