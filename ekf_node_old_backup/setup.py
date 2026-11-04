import os
from glob import glob

from setuptools import find_packages, setup

package_name = "ekf_node"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        # Marker file so ament can find the package (this is why resource/ must exist).
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools", "numpy", "pyyaml"],
    zip_safe=True,
    maintainer="Akshat",
    maintainer_email="you@example.com",
    description="Planar EKF sensor fusion (IMU + rf2o + visual odometry).",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "ekf_localization_node = ekf_node.ekf_localization_node:main",
        ],
    },
)
