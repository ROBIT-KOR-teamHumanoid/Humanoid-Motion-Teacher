cmake_minimum_required(VERSION 3.8)

# Project name
project(Humanoid_Motion_Teacher VERSION 1.0 LANGUAGES CXX)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}")

# Try to find Qt6 first, if not found, try Qt5
find_package(Qt6 COMPONENTS Widgets QUIET)
if(Qt6_FOUND)
    set(QT_VERSION_MAJOR 6)
    message(STATUS "Found Qt6: ${Qt6_VERSION}")
else()
    find_package(Qt5 COMPONENTS Widgets REQUIRED)
    set(QT_VERSION_MAJOR 5)
    message(STATUS "Found Qt5: ${Qt5_VERSION}")
endif()

if(NOT Qt5_FOUND AND NOT Qt6_FOUND)
    message(FATAL_ERROR "Qt5 or Qt6 not found. Please install Qt.")
endif()
find_package(dynamixel_sdk REQUIRED)
find_package(Qwt REQUIRED)
find_package(yaml-cpp REQUIRED)

# Include directories
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_BINARY_DIR} ${dynamixel_sdk_INCLUDE_DIRS})

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} include/dynamixel_sdk/*.h include/Humanoid-Motion-Teacher/*.hpp)

set(SOURCES
    src/main.cpp
    src/dynamixel.cpp
    src/dynamixel_ctrl.cpp
    src/dynamixel_init.cpp
    src/mainwindow.cpp
)

# Generate MOC files
if(QT_VERSION_MAJOR EQUAL 6)
    qt6_wrap_cpp(MOC_SOURCES ${QT_MOC})
    qt6_wrap_ui(UIC_SOURCES ${QT_FORMS})
    qt_add_resources(RESOURCE_SOURCES ${QT_RESOURCES})
else()
    qt5_wrap_cpp(MOC_SOURCES ${QT_MOC})
    qt5_wrap_ui(UIC_SOURCES ${QT_FORMS})
    qt5_add_resources(RESOURCE_SOURCES ${QT_RESOURCES})
endif()

# Add the executable
add_executable(${PROJECT_NAME} ${SOURCES} ${MOC_SOURCES} ${UIC_SOURCES} ${RESOURCE_SOURCES})

# Link the appropriate Qt Widgets library
if(QT_VERSION_MAJOR EQUAL 6)
    target_link_libraries(${PROJECT_NAME} Qt6::Widgets ${dynamixel_sdk_LIBRARIES} Qwt::Qwt)
else()
    target_link_libraries(${PROJECT_NAME} Qt5::Widgets ${dynamixel_sdk_LIBRARIES} Qwt::Qwt)
endif()

target_link_libraries(${PROJECT_NAME} yaml-cpp)

# Platform-specific settings
if(WIN32)
    # Windows-specific settings (if any)

elseif(UNIX)
    # Linux-specific settings (if any)
endif()
