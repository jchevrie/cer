# Copyright (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
# Author: Ugo Pattacini
# email:  ugo.pattacini@iit.it

project(cer_collision-detection)

find_package(YARP 3.2 REQUIRED COMPONENTS OS sig dev math idl_tools manager)

find_package(assimp REQUIRED)
find_package(octomap REQUIRED)
find_package(ccd REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(fcl REQUIRED)

add_executable(${PROJECT_NAME} main.cpp)

target_include_directories(${PROJECT_NAME} PRIVATE ${ASSIMP_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}
    ${YARP_LIBRARIES}
    ${ASSIMP_LIBRARIES}
    ${OCTOMAP_LIBRARIES}
    fcl
    cer_mobile_kinematics)
install(TARGETS ${PROJECT_NAME} DESTINATION bin)

