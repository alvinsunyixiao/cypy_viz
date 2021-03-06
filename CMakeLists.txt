cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# for YCM autocompletion
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Use Gazebo
find_package(gazebo REQUIRED)
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

# Use LCM
find_package(lcm REQUIRED)
find_package(PythonInterp)
if(PYTHONINTERP_FOUND)
    set(python_args PYTHON_SOURCES python_install_sources)
endif()
include(${LCM_USE_FILE})
# Generate LCM Headers and Libraries
lcm_wrap_types(CPP_HEADERS LCM_HEADERS CPP11
               DESTINATION ${CMAKE_BINARY_DIR}/lcmtypes
               ${python_args}
               comm.lcm)
lcm_add_library(liblcm_comm CPP ${LCM_HEADERS})
target_include_directories(liblcm_comm
    INTERFACE ${CMAKE_BINARY_DIR}/lcmtypes ${CMAKE_BINARY_DIR})

# shared library for gazebo model plugin
add_library(model_update SHARED model_update.cc)
target_link_libraries(model_update ${GAZEBO_LIBRARIES} liblcm_comm lcm)
target_include_directories(model_update PRIVATE ${GAZEBO_INCLUDE_DIRS})

# shared library for gazebo world plugin
add_library(world_update SHARED world_update.cc)
target_link_libraries(world_update ${GAZEBO_LIBRARIES} liblcm_comm lcm)
target_include_directories(world_update PRIVATE ${GAZEBO_INCLUDE_DIRS})
