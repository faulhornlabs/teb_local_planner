cmake_minimum_required(VERSION 3.5)
project(teb_local_planner)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_MAP_IMPORTED_CONFIG_RELWITHDEBINFO RELWITHDEBINFO RELEASE)
set(CMAKE_MAP_IMPORTED_CONFIG_MINSIZEREL MINSIZEREL RELEASE)


if(NOT DEFINED CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release) #None, Debug, Release, RelWithDebInfo, MinSizeRel
endif()

#find_package(Boost REQUIRED COMPONENTS system thread graph)
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(g2o CONFIG REQUIRED)
find_package(Eigen3 REQUIRED CONFIG)
if (MSVC)
  find_package(suitesparse CONFIG REQUIRED)
endif()

set(HDRS
    include/teb_local_planner/distance_calculations.h
    include/teb_local_planner/equivalence_relations.h
    include/teb_local_planner/graph_search.h
    include/teb_local_planner/h_signature.h
    include/teb_local_planner/homotopy_class_planner.h
    include/teb_local_planner/homotopy_class_planner.hpp
    include/teb_local_planner/misc.h
    include/teb_local_planner/missing_types.h
    include/teb_local_planner/obstacles.h
    include/teb_local_planner/optimal_planner.h
    include/teb_local_planner/planner_interface.h
    include/teb_local_planner/pose_se2.h
    include/teb_local_planner/recovery_behaviors.h
    include/teb_local_planner/robot_footprint_model.h
    include/teb_local_planner/teb_assert.h
    include/teb_local_planner/teb_config.h
    #include/teb_local_planner/teb_local_planner_ros.h
    include/teb_local_planner/timed_elastic_band.h
    include/teb_local_planner/timed_elastic_band.hpp
    #include/teb_local_planner/visualization.h
    #include/teb_local_planner/visualization.hpp
)

set(HDRS_G2O
    include/teb_local_planner/g2o_types/base_teb_edges.h
    include/teb_local_planner/g2o_types/edge_acceleration.h
    include/teb_local_planner/g2o_types/edge_dynamic_obstacle.h
    include/teb_local_planner/g2o_types/edge_kinematics.h
    include/teb_local_planner/g2o_types/edge_obstacle.h
    include/teb_local_planner/g2o_types/edge_prefer_rotdir.h
    include/teb_local_planner/g2o_types/edge_shortest_path.h
    include/teb_local_planner/g2o_types/edge_time_optimal.h
    include/teb_local_planner/g2o_types/edge_velocity.h
    include/teb_local_planner/g2o_types/edge_velocity_obstacle_ratio.h
    include/teb_local_planner/g2o_types/edge_via_point.h
    include/teb_local_planner/g2o_types/penalties.h
    include/teb_local_planner/g2o_types/vertex_pose.h
    include/teb_local_planner/g2o_types/vertex_timediff.h
)

## Build the teb_local_planner library
set(SRCS
    src/timed_elastic_band.cpp
    src/optimal_planner.cpp
    src/obstacles.cpp
    #src/visualization.cpp
    src/recovery_behaviors.cpp
    src/teb_config.cpp
    src/homotopy_class_planner.cpp
    #src/teb_local_planner_ros.cpp
    src/graph_search.cpp
)

add_library(teb_local_planner ${HDRS} ${HDRS_G2O} ${SRCS})
add_library(teb_local_planner::teb_local_planner ALIAS teb_local_planner)
source_group(src FILES ${SRCS})
source_group(teb_local_planner FILES ${HDRS})
source_group(teb_local_planner\\g2o_types FILES ${HDRS_G2O})
set_target_properties(${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX _d RELWITHDEBINFO_POSTFIX _rd)
target_include_directories(teb_local_planner
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/teb_local_planner>
)

target_link_libraries(teb_local_planner
  PUBLIC
    Eigen3::Eigen
    Boost::system Boost::thread #Boost::graph 
    g2o::core g2o::stuff g2o::types_icp g2o::types_sba
    g2o::solver_csparse
)
if (MSVC)
  target_link_libraries(teb_local_planner PUBLIC SuiteSparse::suitesparseconfig)
endif()


target_compile_options(teb_local_planner PRIVATE $<$<CXX_COMPILER_ID:MSVC>:/bigobj>)

add_executable(test_optim_node src/test_optim_node.cpp)

target_link_libraries(test_optim_node
   teb_local_planner::teb_local_planner
)


#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
install(PROGRAMS
  scripts/cmd_vel_to_ackermann_drive.py
  scripts/export_to_mat.py
  scripts/export_to_svg.py
  scripts/publish_dynamic_obstacle.py
  scripts/publish_test_obstacles.py
  scripts/publish_viapoints.py
  scripts/visualize_velocity_profile.py
  DESTINATION scipts
)

configure_file(cmake/config.cmake.in ${PROJECT_NAME}-config.cmake @ONLY)
install(TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}-targets DESTINATION 
    ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(FILES 
        ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake 
    DESTINATION 
        lib/cmake/${PROJECT_NAME})
    
install(EXPORT ${PROJECT_NAME}-targets NAMESPACE ${PROJECT_NAME}:: DESTINATION lib/cmake/${PROJECT_NAME})


install(FILES
    ${HDRS} DESTINATION include/${PROJECT_NAME}/${PROJECT_NAME}
)
install(FILES
    ${HDRS_G2O} DESTINATION include/${PROJECT_NAME}/${PROJECT_NAME}/g2o_types
)
