from glob import glob

from setuptools import setup

package_name = "piper_real"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="hidayat",
    maintainer_email="mhieda.robotics@gmail.com",
    description="Drive the real Piper arm from MoveIt via topic_based_ros2_control.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "piper_joint_bridge = piper_real.piper_joint_bridge_node:main",
        ],
    },
)
