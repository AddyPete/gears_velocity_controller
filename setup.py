from setuptools import find_packages, setup

package_name = "gears_velocity_controller"

data_files = []
data_files.append(
    ("share/ament_index/resource_index/packages", ["resource/" + package_name])
)
data_files.append(("share/" + package_name, ["package.xml"]))

data_files.append(
    (
        "lib/" + package_name,
        ["gears_velocity_controller/SerialCommunication.py"],
    )
)


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=data_files,
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="minegearscsu",
    maintainer_email="aprlagare1999@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "robot_vel_controller_node = gears_velocity_controller.robot_velocity_controller_node:main"
        ],
    },
)
