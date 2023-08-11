cmake_minimum_required(VERSION 3.5)
project(teb_planner)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(G2O REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(SUITESPARSE REQUIRED)
find_package(Glog REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)


set(dependencies
    rclcpp
    #rclcpp_components
    #sensor_msgs
    geometry_msgs
    nav_msgs
    std_msgs
    tf2
    tf2_ros
)

include_directories(include
                    ${EIGEN3_INCLUDE_DIRS}
                    ${SUITESPARSE_INCLUDE_DIRS}
                    ${GLOG_INCLUDE_DIRS}
                    gtest
                    include/teb_planner
                    )

# aux_source_directory(src          path_smooth_src)

set(teb_lib  
            src/canvas.cpp
            src/optimal_planner.cpp
            src/path_smooth.cpp
            src/thread_pool.cpp
            src/time_elastic_band.cpp
            )

set(EXTERNAL_LIBS
        ${SUITESPARSE_LIBRARIES}
        ${EIGEN3_LIBS}
        ${OpenCV_LIBS}
        gtest
        gtest_main
        glog
        pthread
        )

set(G2O_LIBS
        g2o_cli
        g2o_ext_freeglut_minimal
        g2o_simulator
        g2o_solver_slam2d_linear
        g2o_types_icp
        g2o_types_slam2d
        g2o_core g2o_interface
        g2o_solver_csparse
        g2o_solver_structure_only
        g2o_types_sba
        g2o_types_slam3d
        g2o_csparse_extension
        g2o_opengl_helper
        g2o_solver_dense
        g2o_stuff
        g2o_types_sclam2d
        g2o_parser
        g2o_solver_pcg
        g2o_types_data
        g2o_types_sim3
        cxsparse)

add_executable(test_path_smooth src/test_path_smooth.cpp)
target_link_libraries(test_path_smooth ${EXTERNAL_LIBS} ${G2O_LIBS})
ament_target_dependencies(test_path_smooth geometry_msgs nav_msgs rclcpp std_msgs tf2_msgs tf2_ros)
target_sources(test_path_smooth PUBLIC ${teb_lib})

add_executable(fake_robot_node src/fake_robot_node.cpp)
target_link_libraries(fake_robot_node ${EXTERNAL_LIBS} ${G2O_LIBS})
ament_target_dependencies(fake_robot_node geometry_msgs nav_msgs rclcpp std_msgs tf2_msgs tf2_ros)
target_sources(fake_robot_node PUBLIC ${teb_lib})

add_executable(publisher src/publisher_node.cpp)
ament_target_dependencies(publisher rclcpp std_msgs)

add_executable(subscriber src/subscriber_node.cpp)
ament_target_dependencies(subscriber rclcpp std_msgs)

install(TARGETS
    publisher 
    subscriber
    test_path_smooth
    fake_robot_node
    DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
