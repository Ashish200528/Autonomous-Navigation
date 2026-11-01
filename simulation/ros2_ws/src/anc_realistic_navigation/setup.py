from glob import glob
from setuptools import find_packages, setup

package_name = "anc_realistic_navigation"
setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Ashish",
    maintainer_email="maintainer@example.invalid",
    description="RGB-only traversability and fail-closed safe stopping.",
    license="Apache-2.0",
    entry_points={"console_scripts": [
        "local_traversability_node = anc_realistic_navigation.mapper_node:main",
        "safe_stop_supervisor = anc_realistic_navigation.safe_stop_supervisor:main",
    ]},
)
