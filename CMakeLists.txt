cmake_minimum_required(VERSION 3.1.1)

project(mc_naoqi_dcm)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -w -g")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w -g")
set (CMAKE_CXX_STANDARD 11)
set(ROBOT_NAME "pepper" CACHE STRING "Build with this robot (pepper|nao)")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/CMakeModules)
find_package(qi REQUIRED)
find_package(Boost REQUIRED)
find_package(Boost REQUIRED COMPONENTS 
  filesystem 
  thread 
  program_options
  random
  locale
  chrono
)

message(STATUS "Building with ${ROBOT_NAME} robot")
if("${ROBOT_NAME}" STREQUAL "pepper")
  add_definitions(-DPEPPER)
elseif("${ROBOT_NAME}" STREQUAL "nao")
  add_definitions(-DNAO)
else()
  message( FATAL_ERROR "Only PEPPER and NAO robots are supported" )
endif()

set(SOURCES
    src/mc_naoqi_dcm.cpp
    src/RobotModule.cpp
    src/PepperRobotModule.cpp
)

include_directories(include/ ${QI_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})
add_executable(mc_naoqi_dcm src/main.cpp ${SOURCES})
target_link_libraries(mc_naoqi_dcm ${QI_LIBRARIES} ${Boost_LIBRARIES} crypto)

add_executable(test_qi_dcm utils/check_dcm.cpp ${SOURCES})
target_link_libraries(test_qi_dcm ${QI_LIBRARIES} ${Boost_LIBRARIES} crypto)

install(TARGETS mc_naoqi_dcm
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)
