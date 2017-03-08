cmake_minimum_required(VERSION 2.6)
project(readarduinopsx)

add_executable(readarduinopsx serial_helper.cpp linux_uinput.cpp main.cpp)

install(TARGETS readarduinopsx RUNTIME DESTINATION bin)

SET(CMAKE_CXX_FLAGS "-std=c++1y")

include_directories("/usr/include/glib-2.0/;/usr/lib/x86_64-linux-gnu/glib-2.0/include")

target_link_libraries(readarduinopsx glib-2.0)
target_link_libraries(readarduinopsx pthread)
