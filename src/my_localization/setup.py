from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'my_localization'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bhoobesh',
    maintainer_email='haribhoobesh1234@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'ekf_node = my_localization.ekf_node:main',
        'noisy_odom = my_localization.noisy_odom:main',
        'evaluate = my_localization.evaluate:main'
        ],
    },
)
