cmake_minimum_required(VERSION 3.29)
project(se_cpp_prog_quat_danieloileinik)

set(CMAKE_CXX_STANDARD 14)

include_directories("C:/Users/denoleinik/vcpkg/packages/gtest_x64-mingw-static/include")
include_directories(include)

add_executable(quat
        include/quat_class.h
        src/main.cpp
        src/quat_class.cpp
        tests/test.cpp)
target_link_libraries(quat "C:/Users/denoleinik/vcpkg/packages/gtest_x64-mingw-static/lib/libgtest.a")

