from setuptools import setup
import os 
import glob
package_name = 'hello_world'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/hello_world_launch.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nullbyte',
    maintainer_email='nullbyte.in@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'helloworld = hello_world.helloworld:main'
        ],
    },
)