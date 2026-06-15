from setuptools import find_packages, setup

package_name = 'line_following'

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
    maintainer='kornkawin',
    maintainer_email='kornkawin_th@cmu.ac.th',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'line_follow = line_following.line_follow_node:main',
            'line_guide = line_following.line_guide_node:main',
            'magnetic_reader = line_following.magnetic_reader:main',
        ],
    },
)
