#line 1 "/home/admin/git/shortpod_mustard_robot_arm/.arduino-cli/data/packages/arduino/tools/avr-gcc/7.3.0-atmel3.6.1-arduino7/lib/gcc/avr/7.3.0/plugin/include/omp-low.h"
/* Header file for openMP lowering directives.
   Copyright (C) 2013-2017 Free Software Foundation, Inc.

This file is part of GCC.

GCC is free software; you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free
Software Foundation; either version 3, or (at your option) any later
version.

GCC is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or
FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 for more details.

You should have received a copy of the GNU General Public License
along with GCC; see the file COPYING3.  If not see
<http://www.gnu.org/licenses/>.  */

#ifndef GCC_OMP_LOW_H
#define GCC_OMP_LOW_H

extern tree omp_reduction_init_op (location_t, enum tree_code, tree);
extern tree omp_reduction_init (tree, tree);
extern tree omp_member_access_dummy_var (tree);
extern tree omp_find_combined_for (gimple_stmt_iterator *gsi_p,
				   bool *handled_ops_p,
				   struct walk_stmt_info *wi);


#endif /* GCC_OMP_LOW_H */
