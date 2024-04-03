import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'akari_simulator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
        (
            os.path.join("share", package_name, "urdf"),
            glob(os.path.join("urdf/*")),
        ),
        (
            os.path.join("share", package_name, "rviz"),
            glob(os.path.join("rviz/*")),
        ),
        (
            os.path.join("share", package_name, "image"),
            glob(os.path.join("image/*.png")),
        ),
        (
            os.path.join("share", package_name, "font"),
            glob(os.path.join("font/*.ttf")),
        ),
        (
            os.path.join("share", package_name, "share"),
            glob(os.path.join("akari_simulator/path_generator.py")),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='akariros',
    maintainer_email='mantues@yahoo.co.jp',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "joint_state_publisher = akari_simulator.joint_state_publisher:main",
            "joint_state_publisher_gui_subscriber = akari_simulator.joint_state_publisher_gui_subscriber:main",
            "sim_servo_server = akari_simulator.sim_servo_server:main",
            "sim_servo_client = akari_simulator.sim_servo_client:main",
            "sim_m5_server = akari_simulator.sim_m5_server:main",
            "sim_greeting = akari_simulator.sim_greeting:main",
        ],
    },
)
