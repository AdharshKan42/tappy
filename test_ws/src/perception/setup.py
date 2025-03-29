from setuptools import find_packages, setup

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
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'scipy',
        'opencv-python',
        'paddleocr',
        'typing-extensions',
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


        ],
    },
)
