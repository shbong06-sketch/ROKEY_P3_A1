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
    description="ROS 2 command nodes for smart_farm_nav2_01.usd.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "scene_check = smart_farm_navigation.scene_check:main",
            "path_runner = smart_farm_navigation.path_runner:main",
            "path_runner_smooth = smart_farm_navigation.path_runner_smooth:main",
        ],
    },
)
