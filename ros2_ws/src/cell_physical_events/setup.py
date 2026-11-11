from setuptools import find_packages, setup

package_name = "cell_physical_events"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "pyyaml"],
    zip_safe=True,
    maintainer="Mohammad Nasr",
    maintainer_email="mnasr3@asu.edu",
    description="Physical event vocabulary node for the cell.",
    license="Apache-2.0",
    python_requires=">=3.10",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "physical_events_node = cell_physical_events.node:main",
        ],
    },
)
