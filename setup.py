#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['migrave_behaviour_manager', 'migrave_behaviour_manager_wrapper', 'migrave_game_library'],
    package_dir={'migrave_behaviour_manager': 'common/src/migrave_behaviour_manager',
                 'migrave_behaviour_manager_wrapper': 'ros/src/migrave_behaviour_manager_wrapper',
                 'migrave_game_library': 'ros/src/migrave_game_library'}
)

setup(**d)
