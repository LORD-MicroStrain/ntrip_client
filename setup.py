import os
import glob
from setuptools import setup

package_name = 'ntrip_client'

setup(
    name=package_name,
    version='1.2.0',
    packages=[package_name],
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml', *glob.glob('launch/*')]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Rob Fisher',
    author_email='rob.fisher@parker.com',
    maintainer='Rob Fisher',
    maintainer_email='rob.fisher@parker.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: MIT Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='NTRIP client that will publish RTCM corrections to a ROS topic, and optionally subscribe to NMEA messages to send to an NTRIP server',
    license='MIT License',
    tests_require=['pytest'],
    scripts=[
      'scripts/ntrip_ros.py'
    ]
)