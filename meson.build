#
# Copyright (C) 2023 Patryk Jaworski (blog.regalis.tech)
#
# Author: Patryk Jaworski <regalis@regalis.tech>
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#

project(
  'mcupp',
  'cpp',
  license: 'GPL-3.0-or-later',
  default_options: [
    'buildtype=release',
    'optimization=s',
  ],
  version: '0.1'
)

if not meson.version().version_compare('>=1.3')
  error('This project requires meson>=1.3\n' +
        'Please update your meson installation.\n' +
        'You can use Python\'s venv module to make it easy and non-invasive:\n' +
        '    $ python -m venv venv\n' + 
        '    $ source venv/bin/activate\n' +
        '    $ pip install meson==1.3.0rc2' +
        '\n Happy hacking :-)')
endif

if not meson.is_cross_build()
  error('This project can only be built in a cross build environment.\n' +
        'Please use --cross-file option and pass the selected file' +
        ' from the cross/ directory'
  )
endif

compiler = meson.get_compiler('cpp', native: false)

add_project_arguments(
  compiler.get_supported_arguments([
    '-std=c++23',
    '-g',
    '-Wall',
    '-Wextra',
    '-Wformat=2',
    '-Wshadow',
    '-Wold-style-cast',
    '-Wcast-align',
    '-Wunused',
    '-Woverloaded-virtual',
    '-Wconversion',
    '-Wsign-conversion',
    '-Wmisleading-indentation',
    '-Wduplicated-cond',
    '-Wduplicated-branches',
    '-Wlogical-op',
    '-Wdouble-promotion',
    '-Wnull-dereference',
    '-Wlifetime',
    '-fconcepts-diagnostics-depth=2',
    '-Werror=return-type',
  ]),
  language: ['cpp'],
)

add_project_arguments(
    # TODO: make this a config option
   '-includeinclude/boards/raspberry_pico.hpp',
    language: ['cpp'],
    native: false,
)

include_dirs = include_directories([
  'src/include',
])


subdir('src/')
