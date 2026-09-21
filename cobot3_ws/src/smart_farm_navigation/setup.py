from glob import glob
from setuptools import find_packages, setup


package_name = "smart_farm_navigation"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="rokey",
    maintainer_email="shbong06@gmail.com",
    description="Carter navigation for the smart farm scenes: Nav2 bring-up and /cmd_vel path runners.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "scene_check = smart_farm_navigation.scene_check:main",
            "path_runner = smart_farm_navigation.path_runner:main",
            "path_runner_smooth = smart_farm_navigation.path_runner_smooth:main",
            "navigation_node = smart_farm_navigation.navigation_node:main",
            "scan_sanitizer = smart_farm_navigation.scan_sanitizer:main",
            "go_to_station = smart_farm_navigation.go_to_station:main",
            "nav2_link_check = smart_farm_navigation.nav2_link_check:main",
        ],
    },
)
