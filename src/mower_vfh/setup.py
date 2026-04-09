from setuptools import find_packages, setup

package_name = 'mower_vfh'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/vfh_gazebo.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='q',
    maintainer_email='q@todo.todo',
    description='Depth-camera-based VFH local obstacle avoidance for mower robot',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'depth_vfh_controller = mower_vfh.depth_vfh_controller:main',
        ],
    },
)
