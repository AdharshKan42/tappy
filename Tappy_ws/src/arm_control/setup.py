from setuptools import find_packages, setup

package_name = "arm_control"

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
    maintainer="rover",
    maintainer_email="karenzyc@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "move_arm = arm_control.move_arm:main",
            "tf_subscriber = arm_control.tf_subscriber:main",
            "next_key_push = arm_control.next_key_push:main",
        ],
    },
)
