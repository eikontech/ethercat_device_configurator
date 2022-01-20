# Copyright 2021 Robotic Systems Lab - ETH Zurich:
# Lennart Nachtigall, Jonas Junger
# Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

# 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

cmake_minimum_required(VERSION 3.5)
project(ethercat_device_configurator)

# Add support for C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ethercat_sdk_master REQUIRED)
find_package(rclcpp REQUIRED)

find_package(anydrive)
find_package(opus_ethercat_sdk)
find_package(elmo_ethercat_sdk)
find_package(maxon_epos_ethercat_sdk)
find_package(rokubimini_rsl_ethercat)

set(INCLUDE_DIRS include ${ament_cmake_INCLUDE_DIRS} ${ethercat_sdk_master_INCLUDE_DIRS})
include_directories(${INCLUDE_DIRS})

message(STATUS "INCLUDE_DIRS - ${INCLUDE_DIRS}")

set(LIBRARY_DIRS ${ament_cmake_LIBRARY_DIRS})
link_directories(${LIBRARY_DIRS})

message(STATUS "LIBRARY_DIRS - ${LIBRARY_DIRS}")

set(LIBS ${ament_cmake_LIBRARIES})

message(STATUS "LIBS - ${LIBS}")

find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
  message(STATUS "Using ccache")
  set(CMAKE_CXX_COMPILER_LAUNCHER ${CCACHE_PROGRAM})
  set(CMAKE_C_COMPILER_LAUNCHER ${CCACHE_PROGRAM})
endif()

set(CMAKE_CXX_STANDARD 17)
add_definitions(-Wall -Werror)
add_definitions(-DMELO_USE_COUT)

if(anydrive_FOUND)
  add_definitions(-D_ANYDRIVE_FOUND_)
  set(PACKAGE_DEPENDENCIES ${PACKAGE_DEPENDENCIES} anydrive)
  include_directories(${anydrive_INCLUDE_DIRS})
endif()
if(rokubimini_rsl_ethercat_FOUND)
  add_definitions(-D_ROKUBI_FOUND_)
  set(PACKAGE_DEPENDENCIES ${PACKAGE_DEPENDENCIES} rokubimini_rsl_ethercat)
  include_directories(${rokubimini_rsl_ethercat_INCLUDE_DIRS})
endif()
if(elmo_ethercat_sdk_FOUND)
  add_definitions(-D_ELMO_FOUND_)
  set(PACKAGE_DEPENDENCIES ${PACKAGE_DEPENDENCIES} elmo_ethercat_sdk)
  include_directories(${elmo_ethercat_sdk_INCLUDE_DIRS})
endif()
if(opus_ethercat_sdk_FOUND)
  add_definitions(-D_OPUS_FOUND_)
  set(PACKAGE_DEPENDENCIES ${PACKAGE_DEPENDENCIES} opus_ethercat_sdk)
  include_directories(${opus_ethercat_sdk_INCLUDE_DIRS})
endif()
if(maxon_epos_ethercat_sdk_FOUND)
  add_definitions(-D_MAXON_FOUND_)
  set(PACKAGE_DEPENDENCIES ${PACKAGE_DEPENDENCIES} maxon_epos_ethercat_sdk)
  include_directories(${maxon_epos_ethercat_sdk_INCLUDE_DIRS})
endif()

foreach(sdk ${PACKAGE_DEPENDENCIES})
  message("Found EtherCAT device sdk: " ${sdk})
endforeach()

file(GLOB_RECURSE SRC_FILES ./src/*.c*)

add_library(${PROJECT_NAME} ${SRC_FILES})

target_link_libraries(${PROJECT_NAME} ${anydrive_LIBRARIES}
  ${opus_ethercat_sdk_LIBRARIES} ${elmo_ethercat_sdk_LIBRARIES} ${maxon_epos_ethercat_sdk_LIBRARIES}
  ${rokubimini_rsl_ethercat_LIBRARIES} stdc++fs ${YAML_CPP_LIBRARIES})

add_executable(standalone src/standalone.cpp)
ament_target_dependencies(standalone ethercat_sdk_master rclcpp)
add_dependencies(standalone ${PROJECT_NAME} ${anydrive_EXPORTED_TARGETS}
  ${rokubimini_rsl_ethercat_EXPORTED_TARGETS}
  ${opus_ethercat_sdk_EXPORTED_TARGETS}
  ${elmo_ethercat_sdk_EXPORTED_TARGETS}
  ${maxon_epos_ethercat_sdk_EXPORTED_TARGETS})
target_link_libraries(standalone ${PROJECT_NAME}
  ${anydrive_LIBRARIES}
  ${opus_ethercat_sdk_LIBRARIES} ${elmo_ethercat_sdk_LIBRARIES} ${maxon_epos_ethercat_sdk_LIBRARIES}
  ${rokubimini_rsl_ethercat_LIBRARIES} -pthread stdc++fs ${YAML_CPP_LIBRARIES} yaml-cpp)
  
install(TARGETS ${PROJECT_NAME} standalone
  ARCHIVE
  DESTINATION lib
  LIBRARY
  DESTINATION lib
  RUNTIME
  DESTINATION lib/${PROJECT_NAME})

  install(DIRECTORY include/${PROJECT_NAME}/
          DESTINATION include/${PROJECT_NAME})

# install(DIRECTORY example_config/ DESTINATION share/${PROJECT_NAME}/example_config/)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME}/)

ament_export_dependencies(ament_cmake)
ament_export_include_directories(${INCLUDE_DIRS})
ament_export_libraries(${PROJECT_NAME} ${LIBS})

ament_package()
