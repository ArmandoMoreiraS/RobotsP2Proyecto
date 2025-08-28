from setuptools import setup
package_name = 'palletizer_move'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='armando',
    maintainer_email='you@example.com',
    description='Simple 3x3 palletizer using JointTrajectory',
    license='Apache-2.0',
    entry_points={'console_scripts': ['palletizer = palletizer_move.palletizer_node:main']},
)
