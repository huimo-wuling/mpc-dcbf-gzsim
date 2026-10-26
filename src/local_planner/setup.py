from setuptools import setup

package_name = 'local_planner'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='huimo-wuling',
    maintainer_email='huimo-wuling@users.noreply.github.com',
    description='MPC-D-CBF local planner and controller (ROS 2 port)',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'local_planner = local_planner.local_planner:main',
            'controller = local_planner.controller:main',
        ],
    },
)
