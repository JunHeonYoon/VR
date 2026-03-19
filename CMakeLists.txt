cmake_minimum_required(VERSION 3.15)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
project(VR)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

# -----------------------------
# ROS2 (ament) replacement
# -----------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

# Set CMake to run moc automatically has needed.
set(CMAKE_AUTOMOC ON)
# Set CMake to handle *.ui generation to avoid using QT5_WRAP_UI() macro.
set(CMAKE_AUTOUIC ON)
# Add the build dir to include paths to find UI's headers.
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Find Qt5 libraries.
foreach(module Core Gui Widgets)
    find_package(Qt5 REQUIRED COMPONENTS ${module})
    list(APPEND QT_INCLUDE_DIRS "${Qt5${module}_INCLUDE_DIRS}")
    list(APPEND QT_LIBRARIES    "${Qt5${module}_LIBRARIES}")
    list(APPEND QT_FLAGS        "${Qt5${module}_EXECUTABLE_COMPILE_FLAGS}")
    list(APPEND QT_DEFINITIONS  "${Qt5${module}_DEFINITIONS}")
endforeach()
# Correct QT_FLAGS having semicolons, which break the compile step on POSIX
list(REMOVE_DUPLICATES QT_FLAGS)

# Find OpenVR (find headers + lib)
set(OPENVR_ROOT "${PROJECT_SOURCE_DIR}/thirdparty/openvr" CACHE PATH "Path to OpenVR SDK root")

# header 찾기 (openvr.h)
find_path(OPENVR_INCLUDE_DIR
  NAMES openvr.h
  PATHS
    "${OPENVR_ROOT}/headers"
    "${OPENVR_ROOT}/include"
    "${OPENVR_ROOT}"
  NO_DEFAULT_PATH
)

# lib 찾기 (openvr_api.lib)
if(WIN32)
  find_library(OPENVR_LIBRARY
    NAMES openvr_api
    PATHS
      "${OPENVR_ROOT}/lib"
    PATH_SUFFIXES
      win64 x64 win32 Win64 Win32
    NO_DEFAULT_PATH
  )
else()
  find_library(OPENVR_LIBRARY
    NAMES openvr_api
    PATHS "${OPENVR_ROOT}/lib"
    PATH_SUFFIXES linux64 linux32 osx32
    NO_DEFAULT_PATH
  )
endif()

if(NOT OPENVR_INCLUDE_DIR)
  message(FATAL_ERROR "OpenVR header (openvr.h) not found under: ${OPENVR_ROOT}")
endif()

if(NOT OPENVR_LIBRARY)
  message(FATAL_ERROR "OpenVR library (openvr_api) not found under: ${OPENVR_ROOT}/lib (check win64/win32)")
endif()

message(STATUS "OPENVR_INCLUDE_DIR = ${OPENVR_INCLUDE_DIR}")
message(STATUS "OPENVR_LIBRARY     = ${OPENVR_LIBRARY}")

# -----------------------------
# catkin_package -> ament_export
# -----------------------------
ament_export_include_directories(include)

add_executable(tracker src/main_tracker.cpp src/tracker.cpp )
add_executable(overlay src/main_overlay.cpp src/overlay.cpp ${PROJECT_SOURCE_DIR}/include/overlay.h)


target_include_directories(tracker PRIVATE
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/src
)

target_include_directories(overlay PRIVATE
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/src
)

# include 적용
target_include_directories(tracker PRIVATE "${OPENVR_INCLUDE_DIR}")
target_include_directories(overlay PRIVATE "${OPENVR_INCLUDE_DIR}")

# link 적용
target_link_libraries(tracker "${OPENVR_LIBRARY}" Eigen3::Eigen)
target_link_libraries(overlay "${OPENVR_LIBRARY}" Eigen3::Eigen ${OpenCV_LIBRARIES} ${QT_LIBRARIES})

# -----------------------------
# ROS2 include dirs are usually via ament_target_dependencies,
# but keep your includes unchanged except removing catkin.
# -----------------------------
include_directories(${QT_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

# -----------------------------
# ROS2 dependencies per target
# -----------------------------
ament_target_dependencies(tracker
  rclcpp
  std_msgs
  cv_bridge
  image_transport
)

ament_target_dependencies(overlay
  rclcpp
  std_msgs
  cv_bridge
  image_transport
)

# -----------------------------
# Install + package
# -----------------------------
install(TARGETS tracker overlay 
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
