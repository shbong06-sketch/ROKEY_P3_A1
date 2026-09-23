import os
from glob import glob

from setuptools import find_packages, setup


package_name = "smart_farm_monitor"


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        (
            "share/" + package_name,
            ["package.xml"],
        ),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
        (
            os.path.join("share", package_name, "web"),
            glob("web/*"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="mingja",
    maintainer_email="codesorryy@gmail.com",
    description="Smart farm demonstration observer",
    license="Apache-2.0",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            (
                "cycle_recorder = "
                "smart_farm_monitor.cycle_recorder:main"
            ),
            (
                "dashboard_server = "
                "smart_farm_monitor.dashboard_server:main"
            ),
        ],
    },
)
