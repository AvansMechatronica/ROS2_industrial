from setuptools import find_packages, setup
import os

package_name = 'ros_industrial_install'


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[],
    zip_safe=True,
    maintainer='student',
    maintainer_email='ga.harkema@avans.nl',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
