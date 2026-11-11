from setuptools import find_packages, setup

package_name = "cell_backend_hw"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Mohammad Nasr",
    maintainer_email="mnasr3@asu.edu",
    description="Hardware (UR + RG6) backends for the cell platform interfaces: scaffold.",
    license="Apache-2.0",
    python_requires=">=3.10",
    tests_require=["pytest"],
    entry_points={"console_scripts": []},
)
