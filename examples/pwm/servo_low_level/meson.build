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

example_name = 'pwm_servo_low_level'

examples += executable(
  example_name + '.elf',
  bootloader + files(['main.cpp']),
  include_directories: include_dirs
)

bin_target = example_name + '.bin'
uf2_target = example_name + '.uf2'

examples_bin += custom_target(
  bin_target,
  input: examples[-1],
  output: bin_target,
  command: [cross_objcopy, '-Obinary', '@INPUT@', '@OUTPUT@'],
  build_by_default: true,
)

if regalis_pico_bin2uf2.found()
  examples_uf2 += custom_target(
    uf2_target,
    input: examples_bin[-1],
    output: uf2_target,
    command: [regalis_pico_bin2uf2, '@INPUT@'],
    capture: true,
    build_by_default: true,
  )
endif
