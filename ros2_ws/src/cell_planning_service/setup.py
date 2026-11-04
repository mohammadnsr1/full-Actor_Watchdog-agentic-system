import os
from glob import glob

from setuptools import find_packages
from setuptools import setup

package_name = "cell_planning_service"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Mohammad Nasr",
    maintainer_email="mnasr3@asu.edu",
    description=(
        "The cell's planning service: a backend-neutral ROS 2 service interface "
        "that returns time-parameterized joint trajectories and never executes. "
        "Holds the backend contract, the generic service adapter, and the MoveIt "
        "plan-only backend."
    ),
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "planning_service_node = cell_planning_service.node:main",
        ],
    },
)
