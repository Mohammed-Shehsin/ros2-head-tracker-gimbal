from setuptools import setup

package_name = 'gimbal_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    package_dir={'': 'src'},
    data_files=[
        # Register package with ament index
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),

        # Install package.xml
        ('share/' + package_name, ['package.xml']),

        # Install URDF/Xacro files
        ('share/' + package_name + '/urdf',
         ['urdf/gimbal.urdf.xacro']),

        # Install launch files
        ('share/' + package_name + '/launch',
         ['launch/display.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shehsin',
    maintainer_email='shehsin@example.com',
    description='URDF description of a 2-DOF head-tracker gimbal',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # no Python nodes yet
        ],
    },
)    
