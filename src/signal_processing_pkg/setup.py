from setuptools import find_packages, setup

package_name = 'signal_processing_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['signal_raw.dat']),  # Include your data file
    ],
    install_requires=['setuptools', 'numpy', 'rclpy', 'std_msgs'],
    zip_safe=True,
    maintainer='hali',
    maintainer_email='hali@todo.todo',
    description='A ROS 2 package for signal processing',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lp_filter = signal_processing_pkg.lp_filter:main',
            'signal_source = signal_processing_pkg.signal_source:main',
            'present_data = signal_processing_pkg.present_data:main',
        ],
    },
)
