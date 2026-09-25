#line 1 "/home/admin/git/shortpod_mustard_robot_arm/.arduino-cli/data/packages/arduino/tools/avr-gcc/7.3.0-atmel3.6.1-arduino7/lib/gcc/avr/7.3.0/plugin/include/signop.h"
/* Operations with SIGNED and UNSIGNED.  -*- C++ -*-
   Copyright (C) 2012-2017 Free Software Foundation, Inc.

This file is part of GCC.

GCC is free software; you can redistribute it and/or modify it
under the terms of the GNU General Public License as published by the
Free Software Foundation; either version 3, or (at your option) any
later version.

GCC is distributed in the hope that it will be useful, but WITHOUT
ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
for more details.

You should have received a copy of the GNU General Public License
along with GCC; see the file COPYING3.  If not see
<http://www.gnu.org/licenses/>.  */

#ifndef SIGNOP_H
#define SIGNOP_H

/* This type is used for the large number of functions that produce
   different results depending on if the operands are signed types or
   unsigned types.  The signedness of a tree type can be found by
   using the TYPE_SIGN macro.  */

enum signop {
  SIGNED,
  UNSIGNED
};

#endif
