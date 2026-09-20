import os
from glob import glob

from setuptools import find_packages, setup


package_name = "smart_farm_manager"


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
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="shbong",
    maintainer_email="shbong06@gmail.com",
    description="Smart farm demonstration task manager",
    license="Apache-2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            (
                "task_manager = "
                "smart_farm_manager.task_manager_node:main"
            ),
            (
                "mock_executor = "
                "smart_farm_manager.mock_executor:main"
            ),
        ],
    },
)