from glob import glob
from setuptools import find_packages, setup

package_name = "asp_final_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/config", glob("config/*")),
        (f"share/{package_name}/web", glob("web/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="subin",
    maintainer_email="subin@example.com",
    description="Final ASP mission bringup launch files.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "final_visualization = asp_final_bringup.final_visualization:main",
            "web_dashboard = asp_final_bringup.web_dashboard:main",
        ],
    },
)
