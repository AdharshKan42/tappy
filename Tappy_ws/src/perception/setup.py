from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'perception'
object_detection_root = "perception.detection"


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name, "model"), glob("model/*.*")),
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'scipy',
        'opencv-python',
        'paddleocr',
    ],
    zip_safe=True,
    maintainer='Tappy Team',
    maintainer_email='adharsh.kandula@gmail.com',
    description='Code for autonomous key detection and processing for Tappy',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            f"key_detector = {object_detection_root}.key_detector:main",
            f"mock_baselink = {object_detection_root}.mock_baselink:main",
        ],
    },
)
