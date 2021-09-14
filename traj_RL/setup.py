#!/usr/bin/env python

from distutils.core import setup 
from catkin_pkg.python_setup import generate_distutils_setup 
 
setup_args = generate_distutils_setup( 
packages=['traj_RL', 'SAC'], 
package_dir={'': 'include', '':'include/traj_RL'}, 
) 
setup(**setup_args) 