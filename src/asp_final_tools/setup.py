from glob import glob
from setuptools import find_packages, setup

package_name = "asp_final_tools"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/scripts", glob("scripts/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="subin",
    maintainer_email="subin@example.com",
    description="Final ASP path and runtime audit tools.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "path_audit = asp_final_tools.path_audit:main",
            "final_path_audit = asp_final_tools.final_path_audit:main",
            "mission_timer_csv_logger = asp_final_tools.mission_timer_csv_logger:main",
        ],
    },
)
