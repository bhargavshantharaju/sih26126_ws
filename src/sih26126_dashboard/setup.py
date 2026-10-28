from setuptools import find_packages, setup

package_name = "sih26126_dashboard"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [f"resource/{package_name}"],
        ),
        (
            f"share/{package_name}",
            ["package.xml"],
        ),
    ],
    install_requires=["setuptools", "numpy", "opencv-python"],
    extras_require={
        'test': ['pytest'],
    },
    zip_safe=True,
    tests_require=['pytest'],
    entry_points={
        "console_scripts": [
            "dashboard = sih26126_dashboard.dashboard_node:main",
            "gz_gui_stream = sih26126_dashboard.gui_stream_node:main",
        ],
    },
)
