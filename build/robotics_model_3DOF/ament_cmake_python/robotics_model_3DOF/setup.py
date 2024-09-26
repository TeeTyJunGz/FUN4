from setuptools import find_packages
from setuptools import setup

setup(
    name='robotics_model_3DOF',
    version='0.0.0',
    packages=find_packages(
        include=('robotics_model_3DOF', 'robotics_model_3DOF.*')),
)
