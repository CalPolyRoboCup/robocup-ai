#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
# Copyright © 2019 willdle <willdle@willdle-ThinkPad-X1-Carbon>
#
# Distributed under terms of the MIT license.

"""
Build robocup_master module into ROS stack
"""
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup (
    packages=['robocup_master'],
    package_dir={'':'src'},
)

setup(**setup_args)
