from glob import glob
from setuptools import find_packages, setup

package_name = "asp_final_perception"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="subin",
    maintainer_email="subin@example.com",
    description="Final ASP ArUco perception nodes for UAV exploration and landing.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "aruco_detector = asp_final_perception.aruco_detector:main",
            "detected_marker_csv = asp_final_perception.detected_marker_csv:main",
        ],
    },
)
