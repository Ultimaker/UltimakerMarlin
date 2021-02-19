/*
    Copyright (c) 2019-2021 Ultimaker B.V. All rights reserved.

    Marlin is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Marlin is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Marlin.  If not, see <http://www.gnu.org/licenses/>.
*/
#ifndef CMDLINE_MEM_H                   /* Include file already compiled? */
#define CMDLINE_MEM_H

/* Command implementations */
void            cmdline_initstackprotector(void);

int             cmdline_meminfo         (void   *cmdline,
                                         int    argc,
                                         char   *argv[] __attribute__((unused)));

int             cmdline_memdump         (void   *cmdline,
                                         int    argc,
                                         char   *argv[]);

#endif /* CMDLINE_MEM_H */
