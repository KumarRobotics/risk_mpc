from setuptools import find_packages, setup

package_name = 'negative_obstacle_detection'

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
    maintainer='frankgon',
    maintainer_email='fg1333762@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'noor_detection = negative_obstacle_detection.noor_detection:main',
            'noor_confidence = negative_obstacle_detection.noor_confidence:main',
            'noor_update = negative_obstacle_detection.noor_update:main'
        ],
    },
)
