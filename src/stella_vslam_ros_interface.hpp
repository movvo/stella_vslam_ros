/*
   Copyright 2022 @ AGEVE
   ---------------------------------------------------------
   Authors: Bernat Gaston
   Contact: support.idi@movvo.eu
*/

#ifndef STELLA_SLAM_ROS_INTERFACE_H
#define STELLA_SLAM_ROS_INTERFACE_H

#include <Eigen/Geometry>
#include "rclcpp/rclcpp.hpp"


namespace stella_vslam_ros {
class SystemInterface {
public:
    virtual ~SystemInterface(){}
    virtual void publish_pose(const Eigen::Matrix4d& cam_pose_wc, const rclcpp::Time& stamp) = 0;
    virtual void setParams() = 0;
};
} //namespace stella_vslam_ros

#endif // STELLA_SLAM_ROS_INTERFACE_H