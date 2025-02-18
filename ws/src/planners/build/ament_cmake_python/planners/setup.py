from setuptools import find_packages
from setuptools import setup

setup(
    name='planners',
    version='0.0.0',
    packages=find_packages(
        include=('planners', 'planners.*')),
)
