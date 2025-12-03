from setuptools import find_packages, setup

package_name = 'kinova_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bishal',
    maintainer_email='bishal@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "get_robot_info = kinova_driver.get_robot_info:main",
            "display_color_cam = kinova_driver.display_color_cam:main",
            'joystick_teleop = kinova_driver.joystick_teleop:main',
        ],
    },
)
