from setuptools import setup

package_name = 'ego_controller'

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
    maintainer='Shreyas Muthusamy',
    maintainer_email='shreyas.muthusamy@gmail.com',
    description='The controller of the robot for the final race.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller_node = ego_controller.controller_node:main',
        ],
    },
)
