cmake_minimum_required(VERSION 3.16)
project(ges_voxel_mapping)

option(GES_VOXEL_ENABLE_CATKIN "Enable catkin package metadata if catkin is available" ON)

add_compile_options(-std=c++17 -Wall -Wextra -Wpedantic)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(yaml-cpp REQUIRED)
find_package(catkin QUIET)

if(catkin_FOUND AND GES_VOXEL_ENABLE_CATKIN)
  catkin_package(
    INCLUDE_DIRS include
    DEPENDS EIGEN3 PCL
  )
endif()

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

if(catkin_FOUND AND GES_VOXEL_ENABLE_CATKIN)
  include_directories(${catkin_INCLUDE_DIRS})
endif()

add_library(ges_voxel_core
  src/voxel_analysis.cpp
)

target_compile_definitions(ges_voxel_core PRIVATE
  GES_DEFAULT_CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/config/voxel_morphology.yaml")

target_link_libraries(ges_voxel_core
  ${PCL_LIBRARIES}
  yaml-cpp
)

if(catkin_FOUND AND GES_VOXEL_ENABLE_CATKIN)
  target_link_libraries(ges_voxel_core ${catkin_LIBRARIES})
endif()

add_library(ges_direct_primitive_core
  src/direct_primitive_validation.cpp
)

target_link_libraries(ges_direct_primitive_core
  ges_voxel_core
  ${PCL_LIBRARIES}
  yaml-cpp
)

if(catkin_FOUND AND GES_VOXEL_ENABLE_CATKIN)
  target_link_libraries(ges_direct_primitive_core ${catkin_LIBRARIES})
endif()

add_executable(voxel_morphology_analyzer
  src/voxel_morphology_main.cpp
)

target_compile_definitions(voxel_morphology_analyzer PRIVATE
  GES_DEFAULT_CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/config/voxel_morphology.yaml")

target_link_libraries(voxel_morphology_analyzer
  ges_voxel_core
  ${PCL_LIBRARIES}
  yaml-cpp
)

if(catkin_FOUND AND GES_VOXEL_ENABLE_CATKIN)
  target_link_libraries(voxel_morphology_analyzer ${catkin_LIBRARIES})
endif()

add_executable(voxel_primitive_comparison
  src/voxel_primitive_comparison_main.cpp
)

target_link_libraries(voxel_primitive_comparison
  ges_direct_primitive_core
  ges_voxel_core
  ${PCL_LIBRARIES}
  yaml-cpp
)

if(catkin_FOUND AND GES_VOXEL_ENABLE_CATKIN)
  target_link_libraries(voxel_primitive_comparison ${catkin_LIBRARIES})
endif()

if(catkin_FOUND AND GES_VOXEL_ENABLE_CATKIN)
  install(TARGETS ges_voxel_core ges_direct_primitive_core voxel_morphology_analyzer voxel_primitive_comparison
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

  install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )

  install(DIRECTORY config launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
endif()
