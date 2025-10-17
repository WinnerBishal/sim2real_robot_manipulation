from setuptools import find_packages, setup

package_name = 'kinova_3d_perception'

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
    maintainer_email='addhikari.bishal@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "collect_images_for_intrinsic = kinova_3d_perception.collect_images_for_intrinsic:main",
            "collect_images_and_transform = kinova_3d_perception.collect_images_and_transform:main"
        ],
    },
)
