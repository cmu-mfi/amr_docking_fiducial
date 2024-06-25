from setuptools import find_packages, setup

package_name = 'docking_with_fiducial'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rshah-42',
    maintainer_email='riyashah@andrew.cmu.edu',
    description='Docking package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'docking_with_markers = docking_with_fiducial.docking_with_markers:main'
        ],
    },
)
