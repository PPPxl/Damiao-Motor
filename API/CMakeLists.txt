cmake_minimum_required( VERSION 2.8 )
project( useEigen )

set( CMAKE_BUILD_TYPE "Debug" )
set( CMAKE_CXX_FLAGS_DEBUG "-O0 -Wall -g -ggdb" )

include_directories( "/usr/local/include/" )
link_directories("/usr/local/lib")

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(${PROJECT_SOURCE_DIR}/include)

add_executable(mtct_for_cmpltsub src/mtct_for_cmpltsub.cpp src/pdtMotor.cpp src/handler.cpp src/motionCalculate.cpp)
target_link_libraries(mtct_for_cmpltsub pthread lcm)   # 添加外部链接库pthread, lcm
# add_executable(priThreadTest src/priThreadTest.cpp src/pdtMotor.cpp src/handler.cpp src/motionCalculate.cpp)
# target_link_libraries(priThreadTest pthread lcm)
# add_executable(oneThreadTest2 src/oneThreadTest2.cpp src/pdtMotor.cpp src/handler.cpp src/motionCalculate.cpp)
# target_link_libraries(oneThreadTest2 pthread lcm)
# add_executable(motionCtl src/motionCtl.cpp src/pdtMotor.cpp src/handler.cpp src/motionCalculate.cpp)
# target_link_libraries(motionCtl pthread lcm)
# add_executable(testRecv src/testRecv.cpp)
# target_link_libraries(testRecv pthread lcm)
