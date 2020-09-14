cmake_minimum_required(VERSION 2.8.3)
project(covi_description)

find_package(catkin REQUIRED)

catkin_package(CATKIN_DEPENDS urdf xacro)

install(DIRECTORY meshes
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY urdf
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY robots
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# Generate URDF at build time
catkin_add_gtest(test_urdf_covi test/test_urdf.cpp)
