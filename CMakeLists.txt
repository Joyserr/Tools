cmake_minimum_required(VERSION 3.0.2)
project(TfToolBox)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
  src/tf.cpp
)

target_link_libraries(
    ${PROJECT_NAME}
    ${CMAKE_HOME_DIRECTORY}/libs/libyaml-cpp.a
)