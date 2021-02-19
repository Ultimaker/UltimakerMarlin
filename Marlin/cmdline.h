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
#ifndef CMDLINE_H                       /* Include file already compiled? */
#define CMDLINE_H


/*****************************************************************************/
/*** Macros                                                                ***/
/*****************************************************************************/
#define CMDLINE_LENGTH_MAX      ( 50)   /* Maximum length of a complete command line */
#define CMDLINE_HISTORY_MAX     (  1)   /* Number of lines in history */
#define COMMAND_MAX             ( 12)   /* Maximum length of a command name */
#define ARGS_MAX                (  6)   /* Maximum number of arguments, including command */

#define CMDLINE_MAX             (1)     /* Maximum number of command line interpreters, set to 0 to used the heap */

/* Errors reported by command implementations, triggering standard error messages */
#define ERR_OK                  ( 0)
#define ERR_SYNTAX              (-1)
#define ERR_IO                  (-2)
#define ERR_PARAM               (-3)

#define LOCAL_COMMAND           (-1)    // Mark this gcode line as arriving from the local terminal.

/*****************************************************************************/
/*** Types                                                                 ***/
/*****************************************************************************/
struct command_t {
        char    cmd[COMMAND_MAX];
        int     (*function)(void *cmdline, int argc, char *argv[]);
};


/*****************************************************************************/
/*** Functions                                                             ***/
/*****************************************************************************/
int             cmdline_new             (void                   **cmdline,
                                         const struct command_t *commands);
void            cmdline_delete          (void                   *cmdline);
void            cmdline_thread          (void                   *cmdline);
int             cmdline_printf_P        (void                   *cmdline,
                                         const char             *format,
                                         ...) __attribute__ ((format (printf, 2, 3)));

/* Built-in command line commands */
int             cmdline_echo            (void                   *cmdline,
                                         int                    argc,
                                         char                   *argv[]);
int             cmdline_help            (void                   *cmdline,
                                         int                    argc,
                                         char                   *argv[]);
#if (CMDLINE_HISTORY_MAX > 0)
int             cmdline_history         (void                   *cmdline,
                                         int                    argc,
                                         char                   *argv[]);
#endif /* (CMDLINE_HISTORY_MAX > 0) */


#endif /* CMDLINE_H */
