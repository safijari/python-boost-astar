cmake_minimum_required (VERSION 2.8.11)
project (a-star)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

execute_process(COMMAND python -c
                "import pybind11_cmake; print(pybind11_cmake.__path__[0])"
                OUTPUT_VARIABLE pybind11_DIR OUTPUT_STRIP_TRAILING_WHITESPACE)


FIND_PACKAGE( Boost 1.40 COMPONENTS graph random REQUIRED )
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )
find_package(pybind11 REQUIRED)

pybind11_add_module(astar_cpp PythonInterface.cpp)

TARGET_LINK_LIBRARIES( astar_cpp PRIVATE ${Boost_LIBRARIES} )