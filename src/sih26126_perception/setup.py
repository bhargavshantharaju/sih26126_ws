from setuptools import find_packages, setup

package_name = 'sih26126_perception'

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
    zip_safe=True,
    maintainer='venom',
    maintainer_email='venom@todo.todo',
    description='Camera-based perception for SIH26126.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'perception_baseline = '
            'sih26126_perception.perception_baseline:main',
        ],
    },
)
