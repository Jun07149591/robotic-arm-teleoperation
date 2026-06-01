from glob import glob
from setuptools import setup

package_name = "el_a3_sim"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="EL-A3 Team",
    maintainer_email="dev@example.com",
    description="RViz and ros2_control mock simulation for the EL-A3 robotic arm.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "el_a3_auto_motion = el_a3_sim.auto_motion_node:main",
        ],
    },
)
