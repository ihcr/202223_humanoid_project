# CMakeLists.txt

# License: BSD 3-Clause License
# Copyright (c) 2021, University of Leeds and Harbin Institute of Technology.
# Copyright note valid unless otherwise stated in individual files.
# All rights reserved.

################################################################################
# set up the project
################################################################################
cmake_minimum_required(VERSION 3.13)
project(yamlutils VERSION 1.0.0)

# specify the C++ 17 standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# display all warnings
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
# strongly optimize code
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

# use, i.e. don't skip the full RPATH for the build tree
set(CMAKE_SKIP_BUILD_RPATH FALSE)
# when building, don't use the install RPATH already (but later on when installing)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
list(APPEND CMAKE_INSTALL_RPATH ${CMAKE_INSTALL_PREFIX}/lib)
# add the automatically determined parts of the RPATH which point to directories
# outside the build tree to the install RPATH
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)


################################################################################
# dependencies
################################################################################
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp CONFIG REQUIRED)


################################################################################
# build and install the main library
################################################################################
# headers only library. we only need to install
add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME}
  INTERFACE 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} 
  INTERFACE
    Eigen3::Eigen
    ${YAML_CPP_LIBRARIES}
)
# command to install the library
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
)
# we also need to install the header files
# here we just copy the whole directory
# we might want to only open a few for other packages (replace DIRECTORY by FILES)
install(DIRECTORY include/ DESTINATION include)


################################################################################
# do unit tests
################################################################################
include(CTest)
if(BUILD_TESTING)
    find_package(GTest CONFIG REQUIRED)
    include(GoogleTest)

    add_executable(test_${PROJECT_NAME}
        tests/gtest_main.cpp
        tests/test_yaml_tools.cpp
        tests/test_yaml_eigen.cpp
    )
    target_include_directories(test_${PROJECT_NAME}
      PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    )
    target_link_libraries(test_${PROJECT_NAME}
      PUBLIC
        GTest::gtest
        ${YAML_CPP_LIBRARIES}
        Eigen3::Eigen
    )
    gtest_add_tests(TARGET test_${PROJECT_NAME})
endif()


################################################################################
# create the cmake package
################################################################################
include(CMakePackageConfigHelpers)

# generate the necessary cmake file
set(cm_files "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake")
configure_package_config_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/Config.cmake.in ${cm_files}
  INSTALL_DESTINATION share/${PROJECT_NAME}/cmake)
# we test if there is a version to be installed
if(DEFINED PROJECT_VERSION)
  write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion)
  list(APPEND cm_files
       "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake")
endif()

# we copy the cmake files we would need to configure the project
install(FILES ${cm_files} DESTINATION share/${PROJECT_NAME}/cmake)

# we install the cmake package
install(
  EXPORT ${PROJECT_NAME}Targets
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION share/${PROJECT_NAME}/cmake)