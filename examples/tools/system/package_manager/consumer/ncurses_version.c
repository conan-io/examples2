#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include <ncurses.h>


int main(void) {
    int max_y, max_x;
    char message [256] = {0};

    initscr();
    start_color();
    curs_set(0);
    init_pair(1, COLOR_BLACK, COLOR_WHITE);
    getmaxyx(stdscr, max_y, max_x);

    snprintf(message, sizeof(message), "Conan 2.x Examples - Installed ncurses version: %s - Press any key to exit.\n", curses_version());
    mvprintw(max_y / 2, max_x / 2 - (strlen(message) / 2), "%s", message);

    refresh();
    getch();
    endwin();

    return EXIT_SUCCESS;
}