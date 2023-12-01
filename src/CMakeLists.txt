if(pangolin_viewer_FOUND)
    message(STATUS "Viewer for examples: pangolin viewer")
endif()
if(socket_publisher_FOUND)
    message(STATUS "Viewer for examples: socket publisher")
endif()

set(EXECUTABLE_TARGETS "")

find_package(geo_calicam REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
add_executable(run_slam run_slam.cc stella_vslam_ros.cc)
add_executable(run_slam_offline run_slam_offline.cc stella_vslam_ros.cc)
add_executable(ProducerComponentManager producer_main.cpp producer.cpp stella_vslam_ros.cc)
target_link_libraries(ProducerComponentManager PRIVATE stella_vslam_ros_system)
list(APPEND EXECUTABLE_TARGETS run_slam run_slam_offline ProducerComponentManager)

foreach(EXECUTABLE_TARGET IN LISTS EXECUTABLE_TARGETS)
    # pangolin_viewer is used on a priority basis
    if(pangolin_viewer_FOUND)
        # set macro flag
        target_compile_definitions(${EXECUTABLE_TARGET}
                PRIVATE HAVE_PANGOLIN_VIEWER)
        # link viewer
        target_link_libraries(${EXECUTABLE_TARGET} PRIVATE
                pangolin_viewer::pangolin_viewer)
    endif()
    if(socket_publisher_FOUND)
        # set macro flag
        target_compile_definitions(${EXECUTABLE_TARGET}
                PRIVATE HAVE_SOCKET_PUBLISHER)
        # link viewer
        target_link_libraries(${EXECUTABLE_TARGET} PRIVATE
                socket_publisher::socket_publisher ${SIOCLIENT_LIBRARY} ${PROTOBUF_LIBRARIES})
    endif()

    # setup stack trace logger
    if(USE_STACK_TRACE_LOGGER)
        target_compile_definitions(${EXECUTABLE_TARGET}
                PRIVATE -DUSE_STACK_TRACE_LOGGER)
        target_link_libraries(${EXECUTABLE_TARGET} PRIVATE
                ${GLOG_LIBRARIES})
    endif()

    # setup google-perftools
    if(USE_GOOGLE_PERFTOOLS)
        target_compile_definitions(${EXECUTABLE_TARGET}
                PRIVATE -DUSE_GOOGLE_PERFTOOLS)
        target_link_libraries(${EXECUTABLE_TARGET} PRIVATE
                ${GPERFTOOLS_LIBRARIES})
    endif()

    # setup yaml-cpp
    target_link_libraries(${EXECUTABLE_TARGET} PRIVATE
        yaml-cpp ${Boost_INCLUDE_DIRS})

    if(spdlog_FOUND)
      target_link_libraries(${EXECUTABLE_TARGET} PRIVATE
        spdlog::spdlog)
    endif()

    # build
    target_include_directories(${EXECUTABLE_TARGET}
                               PRIVATE ${rclcpp_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(${EXECUTABLE_TARGET}
                          PRIVATE stella_vslam::stella_vslam ${rclcpp_LIBRARIES} ${OpenCV_LIBRARIES})
    ament_target_dependencies(${EXECUTABLE_TARGET}
                              PUBLIC
                              rclcpp
                              rclcpp_components
                              cv_bridge
                              image_transport
                              message_filters
                              rcutils
                              geometry_msgs
                              nav_msgs
                              sensor_msgs
                              tf2
                              tf2_eigen
                              tf2_geometry_msgs
                              tf2_msgs
                              tf2_ros
                              rosbag2_cpp
                              geo_calicam)
    install(TARGETS ${EXECUTABLE_TARGET}
            DESTINATION lib/${PROJECT_NAME})

endforeach()
