from setuptools import setup

package_name = 'curio_base'

setup(
    name=package_name,
    version='0.2.3',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Manuel Fernández',
    maintainer_email='manolofc@gmail.com',
    description='ROS2 Compatible Hardware controllers for Curio, a Swappy Rover',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'curio_base_controller = curio_base.curio_base_controller:main'
        ],
    },
)
