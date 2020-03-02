cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(gazebo REQUIRED)
link_directories(${GAZEBO_LIBRARY_DIRS})

find_package(lcm REQUIRED)
include(${LCM_USE_FILE})
lcm_wrap_types(CPP_HEADERS LCM_HEADERS CPP11
               DESTINATION ${CMAKE_BINARY_DIR}/lcmtypes
               pose.lcm)
lcm_add_library(liblcm_pose CPP ${LCM_HEADERS})
target_include_directories(liblcm_pose
    INTERFACE ${CMAKE_BINARY_DIR}/lcmtypes ${CMAKE_BINARY_DIR})

list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(model_update SHARED model_update.cc)
target_link_libraries(model_update ${GAZEBO_LIBRARIES} liblcm_pose lcm)
target_include_directories(model_update PRIVATE ${GAZEBO_INCLUDE_DIRS})

