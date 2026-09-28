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
            os.path.join("share", package_name, "config"),
            glob("config/*.yaml"),
        ),
        (
            os.path.join("share", package_name, "sql"),
            glob("smart_farm_monitor/*.sql"),
        ),
        (
            os.path.join("share", package_name, "web"),
            glob("web/*"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="lwh",
    maintainer_email="codesorryy@gmail.com",
    description="Smart farm process recorder and monitoring web service",
    license="Apache-2.0",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "recorder = smart_farm_monitor.recorder_node:main",
            "web = smart_farm_monitor.web_app:main",
        ],
    },
)
