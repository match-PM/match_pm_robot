from setuptools import find_packages, setup

package_name = 'pm_robot_primitive_skills'
submodules = 'pm_robot_primitive_skills/py_modules'

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
    maintainer='pmlab',
    maintainer_email='pmlab',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pm_robot_primitive_skills = pm_robot_primitive_skills.pm_robot_primitive_skills:main'
        ],
    },
)

