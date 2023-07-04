cmake_minimum_required(VERSION 3.15)
project(sdl-example C)

find_package(SDL2 REQUIRED CONFIG)
find_package(SDL2_image REQUIRED CONFIG)
find_package(SDL2_ttf REQUIRED CONFIG)

add_executable(sdl-example src/main.c)

# Main SDL library for init
target_link_libraries(${PROJECT_NAME} PRIVATE SDL2::SDL2main)
if(TARGET SDL2::SDL2-static)
    target_link_libraries(${PROJECT_NAME} PRIVATE SDL2::SDL2-static)
else()
    target_link_libraries(${PROJECT_NAME} PRIVATE SDL2::SDL2)
endif()

# SDL image to make a surface (aka what we'll render)
if(TARGET SDL2_image::SDL2_image-static)
    target_link_libraries(${PROJECT_NAME} PRIVATE SDL2_image::SDL2_image-static)
else()
    target_link_libraries(${PROJECT_NAME} PRIVATE SDL2_image::SDL2_image)
endif()

# SDL ttf so we can display hello world!
if(TARGET SDL2_ttf::SDL2_ttf)
    target_link_libraries(${PROJECT_NAME} PRIVATE SDL2_ttf::SDL2_ttf)
else()
    target_link_libraries(${PROJECT_NAME} PRIVATE SDL2_ttf::SDL2_ttf-static)
endif()

install(TARGETS sdl-example DESTINATION "."
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        )
