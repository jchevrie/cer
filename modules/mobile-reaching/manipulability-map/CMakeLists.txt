# Copyright (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
# Author: Ugo Pattacini
# email:  ugo.pattacini@iit.it

project(cer_manipulability-map)

find_package(octomap REQUIRED)

get_property(R1ModelLib_INCLUDE_DIRS
             TARGET R1ModelLib
             PROPERTY BUILD_INTERFACE_INCLUDE_DIRECTORIES)

get_property(RobotModelLib_INCLUDE_DIRS
             TARGET RobotModelLib
             PROPERTY BUILD_INTERFACE_INCLUDE_DIRECTORIES)

add_executable(${PROJECT_NAME} main.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE ${OCTOMAP_INCLUDE_DIRS})
target_include_directories(${PROJECT_NAME} PRIVATE ${PROJECT_SOURCE_DIR}/include)
target_include_directories(${PROJECT_NAME} PRIVATE ${R1ModelLib_INCLUDE_DIRS})
target_include_directories(${PROJECT_NAME} PRIVATE ${RobotModelLib_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME} RobotModelLib R1ModelLib ${YARP_LIBRARIES} ${OCTOMAP_LIBRARIES} cer_mobile_kinematics)
install(TARGETS ${PROJECT_NAME} DESTINATION bin)

