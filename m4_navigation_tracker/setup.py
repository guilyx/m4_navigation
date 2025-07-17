from setuptools import setup
import os
from glob import glob

package_name = "m4_navigation_tracker"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="wardn",
    maintainer_email="erwin.lejeune@tii.ae",
    description="Pure pursuit path tracking controller for M4 robot",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "m4_navigation_tracker = m4_navigation_tracker.pure_pursuit_controller:main",
        ],
    },
)
