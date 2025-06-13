from setuptools import find_packages, setup

package_name = 'calibration'

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
    maintainer='akannan05',
    maintainer_email='akannan05@ucla.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'save_data = calibration.save_data:main',
            'point_match = calibration.point_match:main',
            'get_extrinsics = calibration.get_extrinsics:main',
            'image_saver = calibration.image_saver:main'
        ],
    },
)
