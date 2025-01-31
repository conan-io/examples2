#include <cstdlib>
#include <cstdio>
#include <ncurses.h>


int main(int argc, char *argv[]) {
    printf("Conan 2.x Examples - Installed NCurses version: %s\n", curses_version());
    return EXIT_SUCCESS;
}