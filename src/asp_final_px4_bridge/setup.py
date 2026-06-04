from setuptools import find_packages, setup

package_name = "asp_final_px4_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="subin",
    maintainer_email="subin@example.com",
    description="Final ASP bridge from asp_final UAV commands to PX4 messages.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "px4_offboard_bridge = asp_final_px4_bridge.px4_offboard_bridge:main",
        ],
    },
)
