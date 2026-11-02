from glob import glob
from setuptools import find_packages, setup
name = "anc_lidar"
setup(
    name=name, version="0.1.0", packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + name]),
        ("share/" + name, ["package.xml"]),
        ("share/" + name + "/launch", glob("launch/*.launch.py")),
        ("share/" + name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"], zip_safe=True,
    maintainer="Ashish", maintainer_email="maintainer@example.invalid",
    description="Provisional realistic planar LiDAR sensing and local mapping.",
    license="Apache-2.0",
    entry_points={"console_scripts":[
        "scan_conditioner = anc_lidar.nodes:scan_conditioner_main",
        "local_scan_mapper = anc_lidar.nodes:local_scan_mapper_main",
        "local_grid_fuser = anc_lidar.nodes:local_grid_fuser_main",
        "lidar_safety_gate = anc_lidar.safety_gate_node:main",
    ]},
)
