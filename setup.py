from setuptools import setup

package_name = 'diff_drive'
gazebo_package = 'gazebo_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'launch/ddrive_rviz.launch.py',
                                   'urdf/ddrive.urdf.xacro', 'config/ddrive_urdf.rviz',
                                   'config/ddrive.yaml', 'launch/ddrive_launch.py',
                                   'worlds/ddrive.world', 'urdf/ddrive.gazebo.xacro',
                                   'config/view_only_ddrive_urdf.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='megsindelar',
    maintainer_email='megansindelar2023@u.northwestern.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drift=diff_drive.drift:drift_entry'
        ],
    },
)
