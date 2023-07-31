from setuptools import find_packages
from setuptools import setup

setup(
    name='commander_server',
    version='0.0.0',
    packages=find_packages(
        include=('commander_server', 'commander_server.*')),
)
