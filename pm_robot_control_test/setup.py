from setuptools import setup

package_name = 'pm_robot_control_test'

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
    maintainer='pmlab',
    maintainer_email='114574121+Florwie@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'pm_robot_forward_position_publisher = \
                    pm_robot_control_test.pm_robot_forward_position_publisher:main'
        ],
    },
)
