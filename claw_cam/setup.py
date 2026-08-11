import os
from glob import glob

from setuptools import find_packages, setup

package_name = "claw_cam"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
        (
            os.path.join("share", package_name, "config"),
            glob(os.path.join("config", "*.yaml"))
            + glob(os.path.join("config", "*.xml")),
        ),
        (
            os.path.join("share", package_name, "config", "calib"),
            glob(os.path.join("config", "calib", "*.yaml")),
        ),
        (
            os.path.join("share", package_name, "scripts"),
            glob(os.path.join("scripts", "*.sh")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Bumblebee Autonomous Systems",
    maintainer_email="advay.pakhale@gmail.com",
    description="Bringup for the AUV4 claw camera on the usb_cam driver",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "camera_autotune_node = claw_cam.camera_autotune_node:main",
            "camera_freeze_watchdog_node = claw_cam.camera_freeze_watchdog_node:main",
        ],
    },
)
