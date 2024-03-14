cmake_minimum_required(VERSION 3.2)

set ( SMOOTHING_EXE_NAME "smoothing" )
project(${SMOOTHING_EXE_NAME})

set(CMAKE_CXX_STANDARD 11)

# Подключение внешних библиотек
include ( "cmake/FindC3D.cmake" )

add_subdirectory ( smoothing )
