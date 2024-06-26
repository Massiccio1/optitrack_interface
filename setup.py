from setuptools import setup

package_name = 'optitrack_interface'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ceron',
    maintainer_email='ceron@todo.todo',
    description='Interface for Optitrack',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		'optitrack = optitrack_interface.optitrack:main',
		'optitrack2 = optitrack_interface.all_in_one:main',
        ],
    },
)
