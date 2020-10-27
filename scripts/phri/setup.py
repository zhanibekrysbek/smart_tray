from setuptools import setup, find_packages
from setuptools.extension import Extension
import numpy as np
import sys

setup(
    name='phri',
    version='1.0',
    description='Library for Physical Human Robot Interaction applications at the University of Illinois at Chicago',
    url='https://github.com/zhanibekrysbek/smart_tray',
    author='Zhanibek Rysbek',
    author_email='zhanibek.rysbek@gmail.com',
    license='MIT',
    packages=find_packages()
)
