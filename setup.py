from setuptools import find_packages, setup
import glob, os

package_name = 'smarc_gps_converters'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test'], include=['smarc_gps_converters', 'smarc_gps_converters.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob.glob('launch/*')),
        (os.path.join('share', package_name, 'config'), glob.glob('config/*')),
        # (os.path.join('lib', package_name), ['smarc_gps_converter_script']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='GPS message converters for SMARC format',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            "smarc_gps_converter = smarc_gps_converters.node:main",
            "gps_to_modem_tf_publisher = smarc_gps_converters.tf_publisher:main",
        ],
    },
)