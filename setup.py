from setuptools import setup

package_name = 'rembrain_gate_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Vasily Morzhakov',
    maintainer_email='morzhakovva@gmail.com',
    description='The package to push/pull streams to/from Rembrain backend',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['gate=rembrain_gate_ros2.main:main_func'],
    },
)
