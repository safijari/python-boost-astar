cmake_minimum_required (VERSION 2.8.11)
project (a-star)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

FIND_PACKAGE( Boost COMPONENTS graph random REQUIRED )
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )
add_subdirectory(pybind11)
pybind11_add_module(astar_cpp PythonInterface.cpp)

TARGET_LINK_LIBRARIES( astar_cpp PRIVATE ${Boost_LIBRARIES} )