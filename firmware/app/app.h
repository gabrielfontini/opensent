/* OpenSENT instrument application (hardware independent).
 * SPDX-License-Identifier: MIT */
#ifndef OPENSENT_APP_H
#define OPENSENT_APP_H

void app_init(void);
void app_line(char *line); /* one command line, writes exactly one reply line */
void app_task(void);       /* call continuously from the main loop */

#endif
