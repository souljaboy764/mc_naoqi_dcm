cmake_minimum_required(VERSION 3.1.1)

project(mc_naoqi_dcm)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -w")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w")
set (CMAKE_CXX_STANDARD 11)

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


set(SOURCES
    src/main.cpp
    src/mc_naoqi_dcm.cpp
    src/RobotModule.cpp
    src/NAORobotModule.cpp
    src/PepperRobotModule.cpp
)

# set(HEADERS
#     include/mc_naoqi_dcm.h
#     include/RobotModule.h
#     include/NAORobotModule.h
#     include/PepperRobotModule.h
# )

include_directories(include/ ${QI_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})
add_executable(mc_naoqi_dcm ${SOURCES})# ${HEADERS})
target_link_libraries(mc_naoqi_dcm ${QI_LIBRARIES} ${Boost_LIBRARIES} crypto)

install(TARGETS mc_naoqi_dcm
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)
