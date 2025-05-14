from setuptools import find_packages, setup

package_name = 'pm_robot_modules'
submodules = 'pm_robot_modules/submodules'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, submodules],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='niklas',
    maintainer_email='terei@match.uni-hannover.de',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pm_robot_modules = pm_robot_modules.pm_robot_modules:main'
        ],
    },
)
