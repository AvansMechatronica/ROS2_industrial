from setuptools import find_packages
from setuptools import setup

setup(
    name='ros_industrial_actuators',
    version='0.1.0',
    packages=find_packages(
        include=('ros_industrial_actuators', 'ros_industrial_actuators.*')),
)
