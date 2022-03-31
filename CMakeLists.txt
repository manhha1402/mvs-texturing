cmake_minimum_required(VERSION 3.1)

LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/elibs/tbb")

project(mvs_texturing)
include(ExternalProject)

if(NOT CMAKE_BUILD_TYPE)
    message(STATUS "Setting build type to 'RELWITHDEBINFO' as none was specified.")
    set(CMAKE_BUILD_TYPE RELWITHDEBINFO)
endif()

set(catkin_RUN_PACKAGES
        roscpp
        roslib
        vision_tools_msgs
)
set(catkin_BUILD_PACKAGES
        ${catkin_RUN_PACKAGES}

)
## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
        ${catkin_BUILD_PACKAGES}
)


#Search for TBB library (release build)
set(CMAKE_BUILD_TYPE_BACKUP ${CMAKE_BUILD_TYPE})
set(CMAKE_BUILD_TYPE RELEASE)
FIND_PACKAGE(TBB REQUIRED)
set(CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE_BACKUP})

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED yes)
add_definitions(-DEIGEN_MPL2_ONLY)
FIND_PACKAGE(OpenMP)
if(OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()
if(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -Wall -Wextra -Wundef -pedantic")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mfpmath=sse -funroll-loops")
endif()
###Open3D############
find_package(Eigen3 REQUIRED NO_MODULE)
if(NOT TARGET Eigen3::Eigen)
    find_package(Threads REQUIRED)
    add_library(Eigen3::Eigen IMPORTED INTERFACE)
    set_property(TARGET Eigen3::Eigen
                 PROPERTY INTERFACE_COMPILE_DEFINITIONS ${EIGEN3_DEFINITIONS})
    set_property(TARGET Eigen3::Eigen
                 PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${EIGEN3_INCLUDE_DIRS})
endif()
find_package(Open3D HINTS ${CMAKE_INSTALL_PREFIX}/lib/CMake)
MESSAGE("Open3D version : ${Open3D_VERSION}")
MESSAGE("Open3D DIR : ${Open3D_DIR}")
list(APPEND Open3D_LIBRARIES dl)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${Open3D_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${Open3D_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${Open3D_EXE_LINKER_FLAGS}")
link_directories(${Open3D_LIBRARY_DIRS})
add_definitions(${Open3D_DEFINITIONS})

###OpenCV from source####
find_package(OpenCV REQUIRED NO_MODULE PATHS /usr/local NO_DEFAULT_PATH)
MESSAGE("OpenCV version : ${OpenCV_VERSION}")
MESSAGE("OpenCV DIR : ${OpenCV_DIR}")

find_package(PNG REQUIRED)
find_package(JPEG REQUIRED)
find_package(TIFF REQUIRED)

catkin_package(
INCLUDE_DIRS

LIBRARIES
        ${PROJECT_NAME}

CATKIN_DEPENDS
        ${catkin_RUN_PACKAGES}
DEPENDS


)










add_subdirectory(elibs)
include_directories(SYSTEM
    ${CMAKE_SOURCE_DIR}/elibs/rayint/libs
    ${CMAKE_SOURCE_DIR}/elibs/mve/libs
    ${CMAKE_SOURCE_DIR}/elibs/mapmap/
    ${CMAKE_SOURCE_DIR}/elibs/mapmap/mapmap
    ${CMAKE_SOURCE_DIR}/elibs/mapmap/ext/dset
    ${Eigen3_INCLUDE_DIRS}
    ${Open3D_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)










include_directories(
    libs
)

link_directories(
    ${CMAKE_SOURCE_DIR}/elibs/mve/libs/mve
    ${CMAKE_SOURCE_DIR}/elibs/mve/libs/util
)

add_subdirectory(libs)
add_subdirectory(apps)
