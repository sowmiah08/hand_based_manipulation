from setuptools import find_packages, setup

package_name = 'system_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/system_bringup']),
        ('share/system_bringup', ['package.xml']),
        ('share/system_bringup/launch', ['launch/apriltag.launch.py', 'launch/full_system.launch.py']),
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
            'tag_to_tf = system_bringup.tag_to_tf:main',
        ],
    },
)
