from setuptools import find_packages, setup

package_name = 'odom_csv_extractor'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'rosbag2_py',
        # Si lidar_slam fournit un module Python installable :
        'lidar_slam',
    ],
    zip_safe=True,
    maintainer='nochi',
    maintainer_email='nochi@todo.todo',
    description='Extrait des topics ROS 2 bag (Odometry & Confidence) vers CSV.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'extract_to_csv = odom_csv_extractor.extract_to_csv:main',
        ],
    },
)

