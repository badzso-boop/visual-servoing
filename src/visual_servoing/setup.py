from setuptools import setup, find_packages
import os
from glob import glob

package_name = "visual_servoing"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        # ROS2 ament resource index regisztráció (kötelező)
        ("share/ament_index/resource_index/packages",
         [f"resource/{package_name}"]),

        # package.xml
        (f"share/{package_name}", ["package.xml"]),

        # Launch fájlok
        (f"share/{package_name}/launch",
         glob("launch/*.launch.py")),

        # Konfiguráció
        (f"share/{package_name}/config",
         glob("config/*")),

        # Gazebo world fájlok
        (f"share/{package_name}/worlds",
         glob("worlds/*.world")),

        # Gazebo modellek
        (f"share/{package_name}/models/connector_socket",
         glob("models/connector_socket/*")),

        (f"share/{package_name}/models/wall",
         glob("models/wall/*")),
    ],
    install_requires=[
        "setuptools",
        "numpy",
        "opencv-python",
    ],
    zip_safe=True,
    maintainer="Ujj Norbert",
    maintainer_email="ujj.norbert@stud.uni-obuda.hu",
    description="IBVS alapú robotkar vezérlő – BSc szakdolgozat",
    license="MIT",
    tests_require=["pytest"],

    # ROS2 node belépési pontok
    # Ezekből lesznek a `ros2 run visual_servoing <executable>` parancsok
    entry_points={
        "console_scripts": [
            "vision_node     = visual_servoing.vision_node:main",
            "controller_node = visual_servoing.controller_node:main",
            "camera_node     = visual_servoing.camera_node:main",
        ],
    },
)
