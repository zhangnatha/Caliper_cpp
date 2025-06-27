cmake_minimum_required(VERSION 3.10)
project(Caliper CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(SOURCES
    main.cpp
    src/edge_width_measure.cpp
    src/CaliperTool.cpp
)

find_package(Threads REQUIRED)

set(THIRDPARTY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/3rdpartyU18")

set(OpenCV_DIR "${THIRDPARTY_DIR}/opencv/lib/cmake/opencv4")
if(NOT EXISTS ${OpenCV_DIR})
    message(FATAL_ERROR "OpenCV path does not exist: ${OpenCV_DIR}")
endif()

find_package(OpenCV REQUIRED)

if(OpenCV_FOUND)
    message(STATUS "OpenCV library status:")
    message(STATUS "    version: ${OpenCV_VERSION}")
    message(STATUS "    libraries: ${OpenCV_LIBS}")
    message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")
else()
    message(FATAL_ERROR "OpenCV not found.")
endif()

include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} ${SOURCES})

target_link_libraries(${PROJECT_NAME}
        PRIVATE
        ${OpenCV_LIBS}
        Threads::Threads
)
