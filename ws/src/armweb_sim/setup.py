import os
from glob import glob

from setuptools import setup

package_name = "armweb_sim"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="thiag",
    description="Headless CPU-only arm simulation for armweb",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "arm_sim_node = armweb_sim.arm_sim_node:main",
        ],
    },
)