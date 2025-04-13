from setuptools import find_packages, setup
from glob import glob

package_name = "keyboard_input"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(),  # This auto-finds Python packages
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="rover",
    maintainer_email="thulasiv2002@gmail.com",
    description="Keyboard input listener for ROS 2",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "keystroke_publish = keyboard_input.keystroke_publish:main",
            "keystroke_listen = keyboard_input.keystroke_listen:main",
            "keystroke_validation = keyboard_input.keystroke_validation:main",
            "keystroke_external = keyboard_input.keystroke_external:main",
        ],
    },
)
