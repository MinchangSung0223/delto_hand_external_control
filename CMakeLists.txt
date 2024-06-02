cmake_minimum_required(VERSION 3.0)
project(delto_hand)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
include(FindPkgConfig)
include(modules/CMakeFunctions.cmake)
set(BUILD_SHARED_LIBS FALSE)
set(CMAKE_MODULE_PATH  ${CMAKE_CURRENT_SOURCE_DIR}/modules)

set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build, options are: Debug | Release | RelWithDebInfo | MinSizeRel" FORCE)
endif()


find_package(SimRobot REQUIRED)
find_package(pinocchio REQUIRED)
find_package(LR REQUIRED)
find_package(ompl REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Bullet REQUIRED)
find_package(urdfdom REQUIRED)
find_package(Threads REQUIRED)
find_package(spdlog REQUIRED)
find_package(ruckig REQUIRED)
find_package(hpp-fcl REQUIRED)
find_package(Boost 1.67 REQUIRED COMPONENTS serialization)


SET(OMPL_INCLUDE_DIRS  "/usr/local/include/ompl-1.6/")

MESSAGE(STATUS "OMPL include dirs are: ${OMPL_INCLUDE_DIRS}")

option(WITH_QT "with QT app" ON)
if(WITH_QT)
    set(CMAKE_INCLUDE_CURRENT_DIR ON)
    find_package(Qt5 COMPONENTS Widgets Core Quick REQUIRED)
    set(CMAKE_AUTOMOC ON)
    set(CMAKE_AUTOUIC ON)
    set(CMAKE_AUTORCC ON)

    set(QTWIDGET_HEADERS_DIR ${PROJECT_SOURCE_DIR}/include/QT/QtWidgets)
    set(DARKSTYLE_HEADERS_DIR ${PROJECT_SOURCE_DIR}/include/QT/DarkStyleWindow)
    include_directories(
        ${QTWIDGET_HEADERS_DIR}
        ${DARKSTYLE_HEADERS_DIR}    
    )
    add_subdirectory(include/QT/DarkStyleWindow)
    add_subdirectory(include/QT/QtWidgets)
    add_subdirectory(include/QT/QCustomPlot_library)
    
    set(QT_LIBRARIES
	sub::QtWidgets
	sub::DarkStyleWindow
	Qt5::Widgets 
	Qt5::Core
	Qt5::Gui
	Qt5::Network
	Qt5::Quick
	qcustomplot
    )
    
    add_compile_definitions(__QT__)
endif()

SET(QT_SRCS 
${CMAKE_CURRENT_SOURCE_DIR}/src/qt/mainwindow.cpp 
${CMAKE_CURRENT_SOURCE_DIR}/src/qt/mainwindow.h 
${CMAKE_CURRENT_SOURCE_DIR}/src/qt/mainwindow.ui 
${CMAKE_CURRENT_SOURCE_DIR}/src/qt/qt_run.cpp 
${CMAKE_CURRENT_SOURCE_DIR}/src/qt/qt_run.h)

SET(CONTROL_SRCS 
${CMAKE_CURRENT_SOURCE_DIR}/src/control/control_run.cpp 
${CMAKE_CURRENT_SOURCE_DIR}/src/control/control_run.h )


SET(HAND_SRCS 
${CMAKE_CURRENT_SOURCE_DIR}/src/delto_hand/Hand.cpp 
${CMAKE_CURRENT_SOURCE_DIR}/src/delto_hand/Hand.h )

add_executable(main src/main.cpp ${RobotSimulator_SRCS} ${QT_SRCS}  ${CONTROL_SRCS} ${HAND_SRCS})
target_link_libraries(main
    Eigen3::Eigen 
    SimRobot::SimRobot 
    LR::Control
    LR::Trajectory            
    Threads::Threads
    spdlog::spdlog
    ompl
    PocoFoundation
    PocoUtil
    PocoNet
    ${BULLET_LIBRARIES} 
    ${urdfdom_LIBRARIES}
    ${QT_LIBRARIES}
    ${OMPL_LIBRARIES}
    ${Boost_LIBRARIES}
)
target_include_directories(main PUBLIC 
    ${RobotSimulator_INCLUDE_DIR}	
    ${OMPL_INCLUDE_DIRS}   
    ${Boost_INCLUDE_DIRS}
)
target_compile_definitions(main PUBLIC -DBT_USE_DOUBLE_PRECISION )