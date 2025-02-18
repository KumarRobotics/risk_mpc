from setuptools import find_packages, setup

package_name = 'planning'

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
    maintainer_email='frankgon@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mpc_planner = planning.mpc_planner_constraints:main',
            'mpc_planner_max_approx_constraint = planning.mpc_planner_max_approx_constraints:main',
            'mpc_planner_max_approx_cost = planning.mpc_planner_max_approx_cost:main',
            'occupancy_grid_parser = planning.occupancy_grid_parser:main'
        ],
    },
)
