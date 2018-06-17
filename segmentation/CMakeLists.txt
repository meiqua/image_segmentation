cmake_minimum_required(VERSION 2.8)
set (CMAKE_CXX_STANDARD 14)
project(cxxSeg_pybind)

OPTION(USE_PYBIND "Enable pybind" FALSE)

#SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_BUILD_TYPE "Release")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb -fPIC")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall -fPIC")


# opencv
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} /home/meiqua/anaconda3/envs/furnace)
set(CUDA_TOOLKIT_ROOT_DIR /usr/local/cuda-8.0)
find_package(OpenCV 3 REQUIRED)
include_directories(${INCLUDE_DIRECTORIES}
        ${OpenCV_INCLUDE_DIRS})


# pybind
if (USE_PYBIND)
    find_package(PythonLibs 3 REQUIRED)
    find_package(pybind11 REQUIRED)
    include(pybind11/Cmake/FindNumPy.cmake)
    set(SOURCE_PYBIND
        pybind11/pybind11.cpp
        pybind11/np2mat/ndarray_converter.cpp)
    include_directories(${INCLUDE_DIRECTORIES}
            ${PYTHON_INCLUDE_DIRS}
            ${PYTHON_NUMPY_INCLUDE_DIR})
endif()


# eigen3
find_package( Eigen3 REQUIRED )
include_directories(${INCLUDE_DIRECTORIES}
    ${EIGEN3_INCLUDE_DIR})


# asp for superpixel
include_directories(${INCLUDE_DIRECTORIES}
    "asp/include"
    "asp")
set(SOURCE_ASP
        asp/src/libasp/algos/ASP.cpp
        asp/src/libasp/algos/DASP.cpp
        asp/src/libasp/algos/SLIC.cpp
        asp/src/libasp/pds/pds.cpp
        asp/src/libasp/pds/Grid.cpp
        asp/src/libasp/pds/FloydSteinberg.cpp)


set(EDCIRCLE_SRC
        edcircle/edcircle.cpp)


# include & src & lib
include_directories(${INCLUDE_DIRECTORIES}
        ${CMAKE_CURRENT_BINARY_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_LIST_DIR})
set(SOURCE
        ${SOURCE_ASP}
        ${EDCIRCLE_SRC}
        seg.cpp
        CIEDE2000/CIEDE2000.cpp)
set(LIBS
    ${OpenCV_LIBS})


# pybind
if (USE_PYBIND)
    pybind11_add_module(${PROJECT_NAME} ${SOURCE} ${SOURCE_PYBIND})
    target_link_libraries(${PROJECT_NAME} PRIVATE ${LIBS})
endif()


add_executable(${PROJECT_NAME}_executable test.cpp ${SOURCE})
target_link_libraries(${PROJECT_NAME}_executable ${LIBS})

