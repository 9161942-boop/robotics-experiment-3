from glob import glob
import os

from setuptools import find_packages, setup

package_name = "mecharm_fixed_pick_sim"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml") + glob("config/*.rviz")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "urdf"), glob("urdf/*.xacro")),
        (os.path.join("share", package_name, "worlds"), glob("worlds/*.world")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="robotics lab",
    maintainer_email="student@example.com",
    description="Fixed-point pick-and-place simulation workflow for mechArm 270.",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "fixed_pick_task = mecharm_fixed_pick_sim.fixed_pick_task:main",
            "manual_control_panel = mecharm_fixed_pick_sim.manual_control_panel:main",
            "auto_sequence_player = mecharm_fixed_pick_sim.auto_sequence_player:main",
            "sorting_scene_monitor = mecharm_fixed_pick_sim.sorting_scene_monitor:main",
            "sorting_task = mecharm_fixed_pick_sim.sorting_task:main",
            "sorting_executor = mecharm_fixed_pick_sim.sorting_executor:main",
        ],
    },
)
