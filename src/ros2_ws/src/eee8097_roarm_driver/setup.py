from glob import glob
import os
from setuptools import find_packages, setup


package_name = "eee8097_roarm_driver"

setup(
    name=package_name,
    version="0.3.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="lester-zed",
    maintainer_email="117880292+lester-zed@users.noreply.github.com",
    description="Guarded RoArm-M2-S state and arm-only trajectory bridge.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "roarm_driver_node = eee8097_roarm_driver.roarm_driver_node:main",
        ],
    },
)
