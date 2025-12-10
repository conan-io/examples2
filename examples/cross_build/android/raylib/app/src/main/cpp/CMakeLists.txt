cmake_minimum_required(VERSION 3.22.1)
project("raylibexample" LANGUAGES C CXX)

set(NATIVE_APP_GLUE_DIR ${ANDROID_NDK}/sources/android/native_app_glue)

find_package(raylib CONFIG REQUIRED)

add_library(${CMAKE_PROJECT_NAME} SHARED)

target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${NATIVE_APP_GLUE_DIR}/android_native_app_glue.c
    native-lib.cpp)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${NATIVE_APP_GLUE_DIR})

target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE
    android
    log
    EGL
    GLESv2
    OpenSLES
    m
    raylib)