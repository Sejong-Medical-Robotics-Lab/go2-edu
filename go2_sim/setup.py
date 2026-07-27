from setuptools import find_packages, setup

package_name = "go2_sim"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    description="Go2 sport mode 관제 노드와 SportClient",
    license="MIT",
    entry_points={
        "console_scripts": [
            "supervisor = go2_sim.supervisor:main",
            "walk_demo = go2_sim.demos.walk_demo:main",
            "waypoint_template = go2_sim.demos.waypoint_template:main",
            "nearest_obstacle = go2_sim.demos.nearest_obstacle:main",
        ],
    },
)
