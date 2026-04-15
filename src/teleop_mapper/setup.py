from setuptools import find_packages, setup

package_name = 'teleop_mapper'

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
    maintainer='zozo',
    maintainer_email='snknitheesh@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pixel_to_3d = teleop_mapper.pixel_to_3d:main',
            '01pixe_to_3d = teleop_mapper.01pixe_to_3d:main',
        ],
    },
)
