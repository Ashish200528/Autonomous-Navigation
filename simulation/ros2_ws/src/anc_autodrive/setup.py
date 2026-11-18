import os
from glob import glob

from setuptools import setup

package_name = "anc_autodrive"


def tree(folder):
    """data_files entries that install a folder recursively under share/."""
    return [(os.path.join("share", package_name, root), [os.path.join(root, f) for f in files])
            for root, _, files in os.walk(folder) if files]


setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        ("share/" + package_name + "/worlds", glob("worlds/*.sdf")),
        ("share/" + package_name + "/config", glob("config/*")),
        *tree("models"),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Autonomous Navigation Car project",
    maintainer_email="maintainer@example.com",
    description="Camera-only autonomous driving demo.",
    license="Apache-2.0",
    entry_points={"console_scripts": [
        "brain=anc_autodrive.brain_node:main",
        "vision=anc_autodrive.vision_node:main",
        "ultrasonic_sim=anc_autodrive.ultrasonic_sim:main",
        "pedestrian_sim=anc_autodrive.pedestrian_sim:main",
    ]},
)
