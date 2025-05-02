from setuptools import setup
import os
from glob import glob

package_name = 'race_stack'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Shreyas Muthusamy',
    maintainer_email='shreyas.muthusamy@gmail.com',
    description='Easy to use stack for launching nodes related to the final race.',
    license='Apache-2.0',
    tests_require=['pytest'],
)