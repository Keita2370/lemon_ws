from setuptools import find_packages, setup

package_name = "go2_piper_joy_teleop"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/joy_teleop.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="hidayat",
    maintainer_email="mhieda.robotics@gmail.com",
    description=(
        "Joystick teleop for the Piper arm (joint_command) and Go2 (cmd_vel), "
        "mode-switched by a hold button."
    ),
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "joy_teleop_node = go2_piper_joy_teleop.joy_teleop_node:main",
        ],
    },
)
