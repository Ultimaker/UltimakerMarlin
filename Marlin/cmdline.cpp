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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <ctype.h>

#include <avr/pgmspace.h>

#include "cmdline.h"
#include "Marlin.h"


/*****************************************************************************/
/*** Macros                                                                ***/
/*****************************************************************************/
#define ISSUE           "\n   __  ______  _                 __\n"             \
                        "  / / / / / /_(_)___ ___  ____ _/ /_____  _____\n"   \
                        " / / / / / __/ / __ '__ \\/ __ '/ //_/ _ \\/ ___/\n" \
                        "/ /_/ / / /_/ / / / / / / /_/ / ,< /  __/ /\n"       \
                        "\\____/_/\\__/_/_/ /_/ /_/\\__,_/_/|_|\\___/_/\n"    \
                        "Welcome to our kingdom. Please don't break anything."
#define PROMPT          "# "

/* Standard ASCII characters */
#define BREAK           0x03
#define BELL            '\a'
#define BACKSPACE       '\b'
#define ESCAPE          0x1b
#define DEL             0x7f


/*****************************************************************************/
/*** Types                                                                 ***/
/*****************************************************************************/
struct cmdline_t {
    const struct command_t  *commands;
#if (CMDLINE_MAX > 0)
    unsigned char           used;
#endif /* (CMDLINE_MAX > 0) */
    unsigned char           localecho;
    char                    lines[1 + CMDLINE_HISTORY_MAX][CMDLINE_LENGTH_MAX + 1];
#if (CMDLINE_HISTORY_MAX > 0)
    char                    escape;
    unsigned char           selected;
#endif /* (CMDLINE_HISTORY_MAX > 0) */
};


/*****************************************************************************/
/*** Globals                                                               ***/
/*****************************************************************************/
#if (CMDLINE_MAX > 0)
struct cmdline_t            cmdlines[CMDLINE_MAX];
#endif /* (CMDLINE_MAX > 0) */


/*****************************************************************************/
/*** Static functions                                                      ***/
/*****************************************************************************/
static int cmd2index(const struct command_t *commands, char *cmd)
{
    int index = 0;

    while (commands[index].function) {
        if (!strncmp (cmd, commands[index].cmd, COMMAND_MAX))
            return index;
        index++;
    }

    return -1;
}


static void proc_line(struct cmdline_t *this_cmdline, char *line)
{
    char          linebuffer[CMDLINE_LENGTH_MAX] = {0};     // CKI: nice to init the array with zero, but overkill since a the first action we do is strncpy which also zeroes the remainder.
    char          *lineptr = &linebuffer[0];
    unsigned int  len = strlen(line);
    int           argc = 0;
    char          *argv[ARGS_MAX];
    int           index;

    /* Copy the line before cutting it up */
    strncpy(lineptr, line, CMDLINE_LENGTH_MAX); // bug CKI: this might result in a string that is not zero terminated

    /* Trim trailing white spaces */
    while (len && (lineptr[len-1] == ' ' || lineptr[len-1] == '\t')) {  // bug CKI: len is not limited to CMDLINE_LENGTH_MAX
        lineptr[len-1] = 0;
        len--;
    }

    /* Build argument list */
    while (*lineptr) {
        /* Skip leading white spaces and replace them with 0-terminations */
//        while ((*lineptr == ' ') || (*lineptr == '\t')) {
        while (isspace(*lineptr)) {
            *lineptr = 0;
            lineptr++;
        }

        /* Store the beginning of this argument */
        if (*lineptr) {
            argv[argc] = lineptr;
            argc++;
        }

        /* Find the separating white space */           // CKI: why?  This seems duplicate code with the start of this loop.
        while (*lineptr && *lineptr != ' ' && *lineptr != '\t')
            lineptr++;
    }

    index = cmd2index(this_cmdline->commands, argv[0]);
    if (index >= 0) {
        int error = this_cmdline->commands[index].function(this_cmdline, argc, argv);

        switch (error) {
        case ERR_OK:
            break;
        case ERR_SYNTAX:
            cmdline_printf_P(this_cmdline, PSTR("Syntax error\n"));
            break;
        case ERR_IO:
            cmdline_printf_P(this_cmdline, PSTR("I/O error\n"));
            break;
        case ERR_PARAM:
            cmdline_printf_P(this_cmdline, PSTR("Parameter error\n"));
            break;
        default:
            cmdline_printf_P(this_cmdline, PSTR("Unknown error (%d)\n"), error);
        }
    } else {
        // Assume the given command is a gcode
        add_command(line, strlen(line), LOCAL_COMMAND);
    }
}


static void proc_char(struct cmdline_t *this_cmdline, char rxd)
{
#if (CMDLINE_HISTORY_MAX > 0)
    if (this_cmdline->escape) {
        if ((this_cmdline->escape == 2) && ((rxd == 'A') || (rxd == 'B'))) {
            int  length = strlen(this_cmdline->lines[CMDLINE_HISTORY_MAX]);

            /* Backspace out old current line */
            if (this_cmdline->localecho)
                while (length) {
                    cmdline_printf_P(this_cmdline, PSTR("%c %c"), BACKSPACE, BACKSPACE);
                    length--;
                }
            /* Change selected line */
            if (rxd == 'A') {
                /* Arrow up */
                if (this_cmdline->selected && strlen(this_cmdline->lines[this_cmdline->selected - 1]))
                    this_cmdline->selected--;
            } else {
                /* Arrow down */
                if (this_cmdline->selected < CMDLINE_HISTORY_MAX)
                    this_cmdline->selected++;
            }
            /* Copy selected line to current line */
            if (this_cmdline->selected != CMDLINE_HISTORY_MAX)
                strncpy(this_cmdline->lines[CMDLINE_HISTORY_MAX], this_cmdline->lines[this_cmdline->selected], CMDLINE_LENGTH_MAX + 1);
            else
                this_cmdline->lines[CMDLINE_HISTORY_MAX][0] = '\0';
            /* Print new current line */
            if (this_cmdline->localecho) {
                cmdline_printf_P(this_cmdline, PSTR("%s"), this_cmdline->lines[CMDLINE_HISTORY_MAX]);
            }
        }
        if (++this_cmdline->escape >= 3)
            this_cmdline->escape = 0;
        return;
    }
#endif /* (CMDLINE_HISTORY_MAX > 0) */

    if (((rxd >= ' ') && (rxd <= '~')) || (rxd == '\t')) {
        int  length = strlen(this_cmdline->lines[CMDLINE_HISTORY_MAX]);

        if (length < CMDLINE_LENGTH_MAX) {
            /* Add readable characters to the line as long as the buffer permits */
            this_cmdline->lines[CMDLINE_HISTORY_MAX][length] = rxd;
            /* Keep string terminated */
            this_cmdline->lines[CMDLINE_HISTORY_MAX][length + 1] = '\0';

            if (this_cmdline->localecho)
                cmdline_printf_P(this_cmdline, PSTR("%c"), rxd);
        } else
            /* Sound the bell is the buffer is full */
            if (this_cmdline->localecho)
                cmdline_printf_P(this_cmdline, PSTR("%c"), BELL);
    } else if (rxd == '\r') {
        if (this_cmdline->localecho)
            cmdline_printf_P(this_cmdline, PSTR("%s"), "\n");

        /* Do not process empty lines */
        if (strlen(this_cmdline->lines[CMDLINE_HISTORY_MAX])) {
            /* Process command line string */
            proc_line(this_cmdline, this_cmdline->lines[CMDLINE_HISTORY_MAX]);

#if (CMDLINE_HISTORY_MAX > 0)
            /* Roll up history if the current line differs from the previous line */
            if (strncmp(this_cmdline->lines[CMDLINE_HISTORY_MAX], this_cmdline->lines[CMDLINE_HISTORY_MAX-1], CMDLINE_LENGTH_MAX + 1)) {
                unsigned char  index;

                for (index = 0; index < CMDLINE_HISTORY_MAX; index++)
                    strncpy(this_cmdline->lines[index], this_cmdline->lines[index + 1], CMDLINE_LENGTH_MAX + 1);
            }
            /* Reset current selection to current line */
            this_cmdline->selected = CMDLINE_HISTORY_MAX;
#endif /* (CMDLINE_HISTORY_MAX > 0) */

            /* Clear new current line */
            this_cmdline->lines[CMDLINE_HISTORY_MAX][0] = '\0';
        }

        if (this_cmdline->localecho)
            cmdline_printf_P(this_cmdline, PSTR(PROMPT));
    } else if ((rxd == BACKSPACE) || (rxd == DEL)) {
        int  length = strlen(this_cmdline->lines[CMDLINE_HISTORY_MAX]);

        /* Delete last character from the line */
        if (length) {
            /* Remove last character from the buffer */
            this_cmdline->lines[CMDLINE_HISTORY_MAX][length-1] = '\0';

            if (this_cmdline->localecho)
                cmdline_printf_P(this_cmdline, PSTR("%c %c"), BACKSPACE, BACKSPACE);
        }
    } else if (rxd == BREAK) {
        /* Ctrl-C character */
        /* Clear current line and reset current selection to current line */
        this_cmdline->lines[CMDLINE_HISTORY_MAX][0] = '\0';
#if (CMDLINE_HISTORY_MAX > 0)
        this_cmdline->selected = CMDLINE_HISTORY_MAX;
#endif /* (CMDLINE_HISTORY_MAX > 0) */
        if (this_cmdline->localecho)
            cmdline_printf_P(this_cmdline, PSTR("^C\n" PROMPT));
#if (CMDLINE_HISTORY_MAX > 0)
    } else if (rxd == ESCAPE) {
        /* Escape character */
        this_cmdline->escape = 1;
#endif /* (CMDLINE_HISTORY_MAX > 0) */
    }
}


/*****************************************************************************/
/*** Functions                                                             ***/
/*****************************************************************************/
int cmdline_new(void **cmdline, const struct command_t *commands)
{
    struct cmdline_t  *this_cmdline;
    int               result = 0;
#if (CMDLINE_MAX > 0)
    int               index = 0;
#endif /* (CMDLINE_MAX > 0) */

#if (CMDLINE_MAX == 0)
    /* Allocate and zero out memory for this new cmdline */
    if (!(this_cmdline = (cmdline_t *)calloc(1, sizeof(struct cmdline_t)))) {     // CKI: ouch! No malloc/calloc in embedded code!!!
        result = -1;
        fprintf_P(stderr, PSTR("Error allocating memory for cmdline\n"));
        goto err_mem;
    }
#else
    /* Find an unused entry for this new cmdline */
    for (;;) {
        /* Increase the used counter first, for thread safety */
        cmdlines[index].used++;
        /*  An unsued entry should be 1 */
        if (cmdlines[index].used == 1)
            break;
        /* Restore the used entries used counter */
        cmdlines[index].used--;

        if (++index >= CMDLINE_MAX) {
            result = -1;
            fprintf_P(stderr, PSTR("Error finding free cmdline\n"));
            goto err_mem;
        }
    }
    this_cmdline = &cmdlines[index];
    memset(this_cmdline, 0, sizeof(struct cmdline_t));
#endif /* (CMDLINE_MAX == 0) */

    /* Initialize new device data */
    this_cmdline->localecho = 1;
    this_cmdline->commands = commands;
#if (CMDLINE_HISTORY_MAX > 0)
    this_cmdline->escape = 0;
    this_cmdline->selected = CMDLINE_HISTORY_MAX;
#endif /* (CMDLINE_HISTORY_MAX > 0) */

    /* Fill out caller's pointer */
    *cmdline = this_cmdline;

    /* Print the banner */
    cmdline_printf_P(this_cmdline, PSTR(ISSUE));

    /* Print the prompt */
    if (this_cmdline->localecho)
        cmdline_printf_P(this_cmdline, PSTR("\n" PROMPT));

    return 0;

err_mem:
    return result;
}


void cmdline_delete(void *cmdline)
{
    struct cmdline_t  *this_cmdline = (struct cmdline_t *)cmdline;

    if (!this_cmdline) {
        fprintf_P(stderr, PSTR("Attempt to delete a non-existing command line"));
        return;
    }

    if (this_cmdline->localecho) {
        int  length = strlen(PROMPT);

#if (CMDLINE_HISTORY_MAX > 0)
        length += strlen(this_cmdline->lines[CMDLINE_HISTORY_MAX]);
#endif
        /* Remove the prompt (and current line) */
        while (length) {
            cmdline_printf_P(this_cmdline, PSTR("%c"), DEL);
            length--;
        }
    }

#if (CMDLINE_MAX == 0)
    /* Free the memory */
    free(this_cmdline);             // CKI: don't use free() in embedded code!
#else
    this_cmdline->used = 0;
#endif /* (CMDLINE_MAX == 0) */
}


void cmdline_thread(void *cmdline)      // CKI: confusing naming, it's not a thread but a processing loop.
{
    struct cmdline_t  *this_cmdline = (struct cmdline_t *)cmdline;
    int               byte;

    if (!this_cmdline) {
        fprintf_P(stderr, PSTR("Attempt to call a non-existing command line thread"));
        return;
    }

    /* Read input data from stdin */
    while ((byte = getchar()) != EOF)
        /* Process the byte */
        proc_char(this_cmdline, (char)byte);
}


int cmdline_printf_P(void *cmdline, const char *format, ...)
{
    struct cmdline_t  *this_cmdline = (struct cmdline_t *)cmdline;
    int               bytes;
    va_list           args;

    if (!this_cmdline) {
        fprintf_P(stderr, PSTR("Attempt to print to a non-existing command line"));
        return -1;
    }

    va_start(args, format);
    bytes = vfprintf_P(stdout, format, args);
    va_end(args);

    return bytes;
}


/*****************************************************************************/
/*** Built-in commands                                                     ***/
/*****************************************************************************/
int cmdline_echo(void *cmdline, int argc, char *argv[])
{
    struct cmdline_t  *this_cmdline = (struct cmdline_t *)cmdline;

    if (argc > 2)
        return ERR_SYNTAX;

    if (argc == 2) {
        if (!strncmp (argv[1], "on", CMDLINE_LENGTH_MAX))
            this_cmdline->localecho = 1;
        else if (!strncmp (argv[1], "off", CMDLINE_LENGTH_MAX))
            this_cmdline->localecho = 0;
        else
            return ERR_SYNTAX;
    }
    cmdline_printf_P(cmdline, PSTR("Echo: %s\n"), this_cmdline->localecho?"on":"off");

    return ERR_OK;
}


int cmdline_help(void *cmdline, int argc, char *argv[] __attribute__((unused)))
{
    struct cmdline_t  *this_cmdline = (struct cmdline_t *)cmdline;
    int               index = 0;

    if (argc > 1)
        return ERR_SYNTAX;

    cmdline_printf_P(cmdline, PSTR("Known commands:\n"));
    while (this_cmdline->commands[index].function) {
        cmdline_printf_P(cmdline, PSTR("%s\n"), /*(char*)*/this_cmdline->commands[index].cmd);
        index++;
    }

    return ERR_OK;
}


#if (CMDLINE_HISTORY_MAX > 0)
int cmdline_history(void *cmdline, int argc, char *argv[] __attribute__((unused)))
{
    struct cmdline_t  *this_cmdline = (struct cmdline_t *)cmdline;
    int               index = 0;
    int               count = 0;

    if (argc > 1)
        return ERR_SYNTAX;

    do {
        if (strlen(this_cmdline->lines[index]))
            cmdline_printf_P(cmdline, PSTR("%d %s\n"), ++count, this_cmdline->lines[index]);
    } while (index++ < CMDLINE_HISTORY_MAX - 1);

    return ERR_OK;
}


#endif /* (CMDLINE_HISTORY_MAX > 0) */
