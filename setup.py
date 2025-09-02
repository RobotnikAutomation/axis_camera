from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'axis_camera'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    # package_dir={'':'axis_camera'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jose Antonio Mendez',
    maintainer_email='jamendez@robotnik.es',
    description='The axis_camera package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'axis_stream_node = axis_camera.axis_stream_node:main',
            'axis_ptz_node = axis_camera.axis_ptz_node:main',
        ],
    },
)
