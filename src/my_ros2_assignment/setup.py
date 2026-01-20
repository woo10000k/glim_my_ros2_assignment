from setuptools import setup, find_packages

package_name = 'my_ros2_assignment'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='woomann',
    maintainer_email='user@example.com',
    description='glim_my_ros2_assignment',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_node = my_ros2_assignment.my_node:main',
        ],
    },
)
