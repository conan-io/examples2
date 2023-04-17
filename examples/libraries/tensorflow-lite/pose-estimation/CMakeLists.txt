cmake_minimum_required(VERSION 3.15)
project(pose-estimation CXX)

find_package(tensorflowlite REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(pose-estimation src/main.cpp)

target_link_libraries(pose-estimation PRIVATE tensorflow::tensorflowlite opencv::opencv)
target_compile_features(pose-estimation PRIVATE cxx_std_17)

install(TARGETS pose-estimation DESTINATION "."
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        BUNDLE DESTINATION bin)

file(GLOB ASSETS "${CMAKE_CURRENT_SOURCE_DIR}/res/*.*")

foreach(asset ${ASSETS})
        add_custom_command(TARGET pose-estimation POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                ${asset}
                $<TARGET_FILE_DIR:pose-estimation>)
endforeach(asset)
