from setuptools import setup

package_name = "anc_navigation"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/anc_navigation"]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/reference_navigation.launch.py"]),
        ("share/" + package_name + "/config", ["config/nav2_reference.yaml", "config/phase6_reference.rviz"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Autonomous Navigation Car project",
    maintainer_email="maintainer@example.com",
    description="Bounded headless Nav2 REFERENCE profile for the custom differential-drive simulator.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "cmd_vel_guard=anc_navigation.cmd_vel_guard:main",
        ],
    },
)
