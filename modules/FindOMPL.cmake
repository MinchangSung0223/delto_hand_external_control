# CMake OMPL config module
#
# Usage:
# find_package(ompl @OMPL_VERSION_MAJOR@ REQUIRED)
# add_library(your_app main.cpp)
# target_link_libraries(your_app PRIVATE ompl::ompl)

@PACKAGE_INIT@

include ("${CMAKE_CURRENT_LIST_DIR}/omplExport.cmake" )
include(CMakeFindDependencyMacro)
find_dependency(Boost REQUIRED COMPONENTS serialization filesystem system)
find_dependency(Eigen3 REQUIRED)

# Add OMPL's find modules to the search path so consumers can also find them.
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR})

if(@OMPL_HAVE_FLANN@) # if(OMPL_HAVE_FLANN)
    find_dependency(flann REQUIRED)
endif()
# If flann is required at version 1.9.2 minimum, then the below logic is unneeded.
#if(flann_FOUND)
#    if(NOT TARGET flann::flann and TARGET flann)
#        add_library(flann::flann ALIAS flan)
#    endif()
#endif()

if(@OMPL_EXTENSION_TRIANGLE@) # if(OMPL_EXTENSION_TRIANGLE)
    # If OMPL was build with TRIANGLE support,
    # then it's required to have TRIANGLE to consume OMPL.
    find_dependency(Triangle REQUIRED)
endif()

if(@Threads_FOUND@) # if(Threads_FOUND)
    find_dependency(Threads QUIET)
endif()

if(@OMPL_HAVE_SPOT@) # if(OMPL_HAVE_SPOT)
    find_dependency(spot MODULE QUIET)
endif()