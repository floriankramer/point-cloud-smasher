cmake_minimum_required(VERSION 3.11)
project(PointCloudSmasher)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PDAL REQUIRED)


file(GLOB SOURCES ${CMAKE_CURRENT_LIST_DIR}/src/*.cpp)
file(GLOB HEADERS ${CMAKE_CURRENT_LIST_DIR}/src/*.h)

add_library(pointcloudsmasher SHARED ${SOURCES} ${HEADERS})
target_include_directories(pointcloudsmasher SYSTEM
  PUBLIC "${CMAKE_CURRENT_LIST_DIR}/godot-cpp/godot_headers/"
  PUBLIC "${CMAKE_CURRENT_LIST_DIR}/godot-cpp/include/"
  PUBLIC "${CMAKE_CURRENT_LIST_DIR}/godot-cpp/include/core/"
  PUBLIC "${CMAKE_CURRENT_LIST_DIR}/godot-cpp/include/gen/"
  PUBLIC ${PDAL_INCLUDE_DIRS})

target_link_directories(pointcloudsmasher
  PUBLIC "${CMAKE_CURRENT_LIST_DIR}/godot-cpp/bin/"
  PUBLIC ${PDAL_LIBRARY_DIRS})
target_link_libraries(pointcloudsmasher
  "godot-cpp.linux.debug.64"
  ${PDAL_LIBRARIES} )
