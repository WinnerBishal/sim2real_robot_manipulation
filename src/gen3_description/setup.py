import os
from glob import glob
from setuptools import find_packages, setup


package_name = 'gen3_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Changed here
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'description'), glob('description/*')),
        (os.path.join('share', package_name, 'meshes/g2f85_collision'), glob('meshes/g2f85_collision/*')),
        (os.path.join('share', package_name, 'meshes/g2f85_visual'), glob('meshes/g2f85_visual/*')),
        (os.path.join('share', package_name, 'meshes/gen3_arm_collision'), glob('meshes/gen3_arm_collision/*')),
        (os.path.join('share', package_name, 'meshes/gen3_arm_visual'), glob('meshes/gen3_arm_visual/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*'))
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
            "cam2ee_tf_pub = gen3_description.cam2ee_tf_pub:main"
        ],
    },
)
