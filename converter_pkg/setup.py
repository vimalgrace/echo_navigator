from setuptools import find_packages, setup

package_name = 'converter_pkg'

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
    maintainer='magic',
    maintainer_email='vimalgracerobotics@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "nav2_cmd_to_arduino = converter_pkg.nav2_cmd_to_arduino:main",
            "scan_raw_to_scan = converter_pkg.scan_filter:main",
             "imu_publisher = converter_pkg.imu_data_publisher:main"
        ],
    },
)
