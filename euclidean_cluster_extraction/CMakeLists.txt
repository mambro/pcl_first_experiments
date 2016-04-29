cmake_minimum_required(VERSION 2.8)
project(euclidean_cluster_extraction CXX)

set(CMAKE_CXX_FLAGS   "${CMAKE_CXX_FLAGS} -std=c++11") 
set(CMAKE_CXX_FLAGS   "${CMAKE_CXX_FLAGS} -march=native")
set(CMAKE_BUILD_TYPE  Release)
## set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/modules")
## set(INCLUDE           "${CMAKE_SOURCE_DIR}/include")


find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DWITH_PCL")
find_package(PCL REQUIRED)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
include_directories(${PCL_INCLUDE_DIRS})

add_executable(cluster_extraction     cluster_extraction.cpp)
target_link_libraries(cluster_extraction ${PCL_LIBRARIES} )

