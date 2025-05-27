cmake_minimum_required(VERSION 3.25)
project(monorepo CXX)

include(FetchContent)

function(add_project SUBFOLDER)
    FetchContent_Declare(
        ${SUBFOLDER}
        SOURCE_DIR ${CMAKE_CURRENT_LIST_DIR}/../${SUBFOLDER}
        SYSTEM
        OVERRIDE_FIND_PACKAGE
    )
    FetchContent_MakeAvailable(${SUBFOLDER})
endfunction()


add_project(say)
# They should be defined in the say/CMakeLists.txt, but we can fix it here
add_library(say::say ALIAS say)
add_project(hello)
