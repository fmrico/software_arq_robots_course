#!/usr/bin/env python

from distutils.core import setup

from catkin_pkg.python_setup import generate_distutils_setup


d = generate_distutils_setup(
    packages=['plansys2_tools'],
    package_dir={'': 'src'},
    scripts=['scripts/rqt_plansys2_performers']
)

setup(**d)

