from setuptools import setup
from glob import glob
import os


package_name = 'curio_base'

setup(
    name=package_name,
    version='0.2.3',
    package_dir={'': 'src'},
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    py_modules = ['curio_base.base_controller', 'curio_base.base_failsafe', 'curio_base.lx16a_driver', 'curio_base.lx16a_encoder_filter'], 
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Manuel Fernández',
    maintainer_email='manolofc@gmail.com',
    description='ROS2 Compatible Hardware controllers for Curio, a Swappy Rover',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    scripts=[ 'scripts/lx16a_cmd_base.py', 'scripts/lx16a_cmd_vel_random.py', 'scripts/lx16a_cmd_vel_sinusoid.py', 'scripts/lx16a_cmd_vel_stepped.py', 'scripts/lx16a_driver_test.py', 'scripts/lx16a_encoder_filter_test.py', 'scripts/lx16a_encoder_logger.py', 'scripts/lx16a_failsafe_test.py', 'scripts/lx16a_mean_filter_test.py', 'scripts/lx16a_odometry_test.py', 'scripts/lx16a_train_classifier.py', 'scripts/lx16a_train_regressor.py'],
    entry_points={
        'console_scripts': [
            'curio_base_controller = curio_base.curio_base_controller:main',
            'curio_base_failsafe = curio_base.curio_base_failsafe:main'            
        ],
    },
)
