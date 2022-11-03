cmake_minimum_required( VERSION 2.8 )
project( useEigen )

set( CMAKE_BUILD_TYPE "Debug" )
set( CMAKE_CXX_FLAGS_DEBUG "-O0 -Wall -g -ggdb" )

#include_directories( "/usr/local/include/eigen3" )

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(${PROJECT_SOURCE_DIR}/include)

add_executable(SendTest src/SendTest.cpp src/pdtMotor.cpp)
target_link_libraries(SendTest pthread)   #添加外部链接库pthread
#add_executable(ReceiveTest src/ReceiveTest.cpp src/pdtMotor.cpp)