cmake_minimum_required(VERSION 3.28)
project(hse_cpp_task)

set(CMAKE_CXX_STANDARD 20)

add_executable(hse_cpp_task main.cpp)


if(DEFINED TYPES)
 add_compile_definitions("-DTYPES=${TYPES}")
 if(DEFINED SIZES)
   add_compile_definitions("-DSIZES=${SIZES}")
 endif(DEFINED SIZES)
 add_executable(main main.cpp)
endif(DEFINED TYPES)