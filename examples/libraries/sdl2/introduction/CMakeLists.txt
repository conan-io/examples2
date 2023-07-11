cmake_minimum_required(VERSION 3.15)
project(sdl-example C)

find_package(SDL2 REQUIRED CONFIG)
find_package(SDL2_image REQUIRED CONFIG)
find_package(SDL2_ttf REQUIRED CONFIG)

add_executable(sdl-example src/main.c)

# Main SDL library for init
target_link_libraries(${PROJECT_NAME} PRIVATE SDL2::SDL2main SDL2::SDL2-static)

# SDL image to make a surface (aka what we'll render)
target_link_libraries(${PROJECT_NAME} PRIVATE SDL2_image::SDL2_image-static)

# SDL ttf so we can display hello world!
target_link_libraries(${PROJECT_NAME} PRIVATE SDL2_ttf::SDL2_ttf)

