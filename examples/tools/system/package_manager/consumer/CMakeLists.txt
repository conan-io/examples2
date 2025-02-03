cmake_minimum_required(VERSION 3.15)
project(ncurses_version C)

find_package(ncurses CONFIG REQUIRED)

add_executable(${PROJECT_NAME} ncurses_version.c)
target_link_libraries(${PROJECT_NAME} PRIVATE ncurses::ncurses)

install(TARGETS ${PROJECT_NAME} DESTINATION bin)