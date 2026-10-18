from glob import glob
from setuptools import find_packages, setup


package_name = "krowd_stanley_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/rviz", glob("rviz/*.rviz")),
        # `ros2 run krowd_stanley_control record_stanley_tuning.sh` /
        # `ros2 run krowd_stanley_control run_stanley.sh check|drive`
        ("lib/" + package_name, glob("scripts/*.sh") + glob("scripts/*.py")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="krowd",
    maintainer_email="krowd2026@gmail.com",
    description="Stanley path tracking against a mapped route.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "route_publisher_node = krowd_stanley_control.route_publisher_node:main",
            "vehicle_state_node = krowd_stanley_control.vehicle_state_node:main",
            "stanley_node = krowd_stanley_control.stanley_node:main",
            "vehicle_dynamics_sim_node = krowd_stanley_control.vehicle_dynamics_sim_node:main",
            "avoidance_supervisor_node = krowd_stanley_control.avoidance_supervisor_node:main",
        ],
    },
)
