cmake_minimum_required(VERSION 3.5)

project(bno055_cpp)

find_program(CCACHE_FOUND ccache)
if(CCACHE_FOUND)
    set(CMAKE_CXX_COMPILER_LAUNCHER ccache)
endif(CCACHE_FOUND)

set(CMAKE_CXX_STANDARD 17)

add_compile_options(-Wall -Wextra -O3)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

set(CMAKE_CXX_CPPCHECK "cppcheck" "--enable=all")
include_directories(
    ./ext/serial/include
    ./src
    )

add_subdirectory(ext/serial)


add_library(bno_lib
    src/BNO055.cpp
    src/BNO055.h
    src/BNO055DataDefinition.h
    src/BNO055CommandDefinitions.h
    )

target_link_libraries(bno_lib
    Threads::Threads
)

add_executable(bno_test
    src/BNO055Test.cpp

)

target_link_libraries(bno_test
    serial
    bno_lib
)
