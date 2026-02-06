from setuptools import find_packages
from setuptools import setup

setup(
    name='turtlebot4_msgs',
    version='2.1.1',
    packages=find_packages(
        include=('turtlebot4_msgs', 'turtlebot4_msgs.*')),
)
