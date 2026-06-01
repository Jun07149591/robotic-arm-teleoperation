import os
from glob import glob
from setuptools import setup

package_name = "el_a3_teleop"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "xbox_teleop_node = el_a3_teleop.xbox_teleop_node:main",
            "pico_teleop_node = el_a3_teleop.pico_teleop_node:main",
        ],
    },
)
