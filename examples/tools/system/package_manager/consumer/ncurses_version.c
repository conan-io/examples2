#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include <ncurses.h>


int main(void) {
    int max_y, max_x;
    char message [256] = {0};

    initscr();

    start_color();
    init_pair(1, COLOR_BLUE, COLOR_WHITE);
    getmaxyx(stdscr, max_y, max_x);

    snprintf(message, sizeof(message), "Conan 2.x Examples - Installed ncurses version: %s\n", curses_version());
    attron(COLOR_PAIR(1));
    mvprintw(max_y / 2, max_x / 2 - (strlen(message) / 2), "%s", message);
    attroff(COLOR_PAIR(1));

    refresh();

    return EXIT_SUCCESS;
}