import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'vision_mapping'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
    	(os.path.join('share', package_name, 'config'), glob('config/*')),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pascal',
    maintainer_email='p.aaldijk@student.avans.nl',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'listener = src.occupancyGrid:main', 
        ],
    },
)
