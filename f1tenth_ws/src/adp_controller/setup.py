from setuptools import setup
import os
from glob import glob

package_name = 'adp_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # Package manifest
        ('share/' + package_name, ['package.xml']),

        # Launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),

        # Include gain table .npy files
        (os.path.join('share', package_name, 'scripts/utils'), glob('scripts/utils/*.npy')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Murali Peddi',
    maintainer_email='mp6904@nyu.edu',
    description='ADP-based Controller for F1TENTH',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'frenet_error_node = adp_controller.frenet_error_node:main',
            'adp_raceline_control = adp_controller.adp_raceline_control:main',
            'raceline_publisher = adp_controller.raceline_publisher:main',
        ],
    },
)

