# -----------------------------------------------------------------------------
#
# Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
# SPDX-License-Identifier: BSD-3-Clause
#
# -----------------------------------------------------------------------------

from setuptools import find_packages, setup

package_name = "icm20948"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="quic-salvi",
    maintainer_email="quic_salvi@quicinc.com",
    description="ROS2 driver for ICM20948 IMU",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["icm20948 = icm20948.icm20948:main"],
    },
)
