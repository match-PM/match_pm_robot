from setuptools import find_packages, setup

package_name = 'pm_opcua_skills_unity'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='match-mover',
    maintainer_email='wiemann@match.uni-hannover.de',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'opcua_skills_unity = pm_opcua_skills_unity.opcua_skills_unity:main'
        ],
    },
)
