cmake_minimum_required(VERSION 4.4 FATAL_ERROR)

# Detect CPU cores and set parallel build level to half
include(ProcessorCount)
ProcessorCount(N)
if(NOT N EQUAL 0)
    math(EXPR PARALLEL_JOBS "${N} / 2")
    if(PARALLEL_JOBS LESS 1)
        set(PARALLEL_JOBS 1)
    endif()
    set(CMAKE_BUILD_PARALLEL_LEVEL ${PARALLEL_JOBS})
    message(STATUS "Detected ${N} CPU cores, parallel builds will use ${PARALLEL_JOBS} jobs")
    message(STATUS "Use 'cmake --build .' or 'make -j${PARALLEL_JOBS}' for parallel compilation")
else()
    message(STATUS "Could not detect CPU cores, defaulting to sequential builds")
endif()

# Modern C++ standard (PCL 1.15 requires C++17 or higher)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Compiler warnings
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

project(lidar_obstacle_detection VERSION 1.0.0 LANGUAGES CXX)

find_package(PCL 1.15 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} src/ransac)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

add_executable(environment
    src/environment.cpp
    src/render/render.cpp
    src/processPointClouds.cpp
)
target_link_libraries(environment ${PCL_LIBRARIES})

# Quiz executables
add_executable(quizCluster
    src/cluster.cpp
    src/render/render.cpp
)
target_link_libraries(quizCluster ${PCL_LIBRARIES})

add_executable(quizRansac
    src/ransac2d.cpp
    src/render/render.cpp
    src/processPointClouds.cpp
)
target_link_libraries(quizRansac ${PCL_LIBRARIES})

