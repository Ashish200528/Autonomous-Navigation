from setuptools import setup

package_name = "anc_localization"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/anc_localization"]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/localization.launch.py"]),
        ("share/" + package_name + "/config", ["config/ekf.yaml", "config/wheel_odometry.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Autonomous Navigation Car project",
    maintainer_email="maintainer@example.com",
    description="Planar EKF localization and covariance-qualified wheel odometry.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "wheel_odometry_covariance=anc_localization.wheel_odometry_covariance:main",
        ],
    },
)
