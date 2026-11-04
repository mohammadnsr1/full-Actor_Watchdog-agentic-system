from setuptools import find_packages
from setuptools import setup

package_name = "cell_planning_curobo"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "INSTALL.md"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Mohammad Nasr",
    maintainer_email="mnasr3@asu.edu",
    description="cuRobo backend for the cell planning service (own Python environment).",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "curobo_planning_node = cell_planning_curobo.node:main",
        ],
    },
)
