from setuptools import setup

package_name = "anc_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=["anc_bringup"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/anc_bringup"]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/sim.launch.py"]),
        ("share/" + package_name + "/config", ["config/bridge.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Autonomous Navigation Car project",
    maintainer_email="maintainer@example.com",
    description="Headless simulation launch and safe sensor interfaces.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "imu_sanitizer=anc_bringup.imu_sanitizer:main",
        ],
    },
)
