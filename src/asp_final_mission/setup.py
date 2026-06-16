from setuptools import find_packages, setup

package_name = "asp_final_mission"

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
    description="Final ASP mission supervisor using only asp_final topics.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "mission_supervisor = asp_final_mission.mission_supervisor:main",
        ],
    },
)
