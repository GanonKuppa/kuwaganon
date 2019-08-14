cmake_minimum_required(VERSION 2.8)

# message(cmake -G "MinGW Makefiles" ..) windowsでは以下でcmakeを叩くこと

set(CMAKE_C_COMPILER "C:/Program Files/mingw-w64/x86_64-8.1.0-posix-sjlj-rt_v6-rev0/mingw64/bin/gcc")
set(CMAKE_CXX_COMPILER "C:/Program Files/mingw-w64/x86_64-8.1.0-posix-sjlj-rt_v6-rev0/mingw64/bin/g++")

set(CMAKE_CXX_STANDARD, 11)

include_directories(C:/dropbox/Dropbox/My_data/Umouse/program/trape_test_python/)
add_executable(main main.cpp udp.cpp curveFactory.cpp)
target_link_libraries(main wsock32 ws2_32)
 