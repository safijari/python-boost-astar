# Copyright 2023 Jariullah Safi

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.

# You should have received a copy of the GNU Lesser General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

from setuptools import setup, find_packages
try:
    import pybind11_cmake
except ImportError:
    print("pybind11-cmake must be installed."
          "Try \n \t pip install pybind11_cmake")
    import sys
    sys.exit()

from pybind11_cmake import CMakeExtension, CMakeBuild

setup(
    name='python-boost-astar',
    version='0.1.0',
    author='Jariullah Safi',
    author_email='safijari@isu.edu',
    description=
    'A very barebones wrapper over the example for boost-astar',
    long_description='',
    packages=find_packages(),
    install_requires=[
        'numpy', 'tiny_tf', 'numba'
    ],
    setup_requires=['pybind11_cmake'],
    ext_modules=[CMakeExtension('astar_cpp')],
    cmdclass=dict(build_ext=CMakeBuild),
    zip_safe=False,
)
