from setuptools import find_packages, setup

package_name = "drone_gesture"

setup(
    name=package_name,
    version="0.3.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", [
            "launch/gesture_system.launch.py",
            "launch/gesture_test.launch.py",
            "launch/gesture_full.launch.py",
        ]),
        ("share/" + package_name + "/config", [
            "config/gesture_params.yaml",
        ]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="dev@example.com",
    description="Drone gesture recognition control system - MediaPipe + MAVROS",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "gesture_recognizer = drone_gesture.gesture_recognizer:main",
            "gesture_commander = drone_gesture.gesture_commander:main",
            "safety_monitor = drone_gesture.safety_monitor:main",
            "diagnostics = drone_gesture.diagnostics:main",
            "gesture_velocity_controller = drone_gesture.gesture_velocity_controller:main",
        ],
    },
)
