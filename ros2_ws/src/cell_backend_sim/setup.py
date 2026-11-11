from setuptools import find_packages, setup

package_name = "cell_backend_sim"

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
    description="Isaac Sim backends for the cell platform interfaces.",
    license="Apache-2.0",
    python_requires=">=3.10",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "ground_truth_pose_publisher = cell_backend_sim.ground_truth_pose_publisher:main",
        ],
    },
)
