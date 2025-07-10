from setuptools import find_packages, setup
import os, glob

package_name = 'vicpinky_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob(os.path.join('launch', '*launch.*'))),
        ('share/' + package_name + '/config', glob.glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mkh',
    maintainer_email='kyung133851@pinklab.art',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "bringup=vicpinky_bringup.bringup:main"
        ],
    },
)
