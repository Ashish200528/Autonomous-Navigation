from setuptools import find_packages, setup
from glob import glob
import os

package_name = "anc_perception"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Ashish",
    maintainer_email="ashish@example.invalid",
    description="Conservative RGB-only FREE/OCCUPIED/UNKNOWN image perception.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "rgb_perception_node = anc_perception.perception_node:main",
        ],
    },
)
