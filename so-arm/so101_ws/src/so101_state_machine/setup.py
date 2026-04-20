from setuptools import find_packages, setup

package_name = 'so101_state_machine'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ezhilan Veluchami',
    maintainer_email='ezhilan.veluchami@gmail.com',
    description='SO101 pick-and-place BT with MoveIt2 and RGB-D perception',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'bt_node = so101_state_machine.bt_node:main',
            'cup_detector = so101_state_machine.cup_detector:main',
        ],
    },
)
