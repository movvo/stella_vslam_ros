#ifndef GEO_VSLAM__PRODUCER_HPP
#define GEO_VSLAM__PRODUCER_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/component_manager.hpp>
#include "rclcpp_components/component_manager_isolated.hpp"
#include "geo_calicam/monocular_calicam.hpp"

#include <system.h>
#include <stella_vslam_ros.h>
#include <stella_vslam/data/frame.h>

#include <opencv2/opencv.hpp>
#include <boost/circular_buffer.hpp>

class Video {
public:
    Video(std::string path_to_video);
    ~Video();
    bool GrabFrame(cv::Mat & frame);
private:
    cv::VideoCapture cap_;
};

class Producer : public stella_vslam_ros::System, public rclcpp_components::ComponentManagerIsolated<rclcpp::executors::SingleThreadedExecutor> {
public:
    using rclcpp_components::ComponentManagerIsolated<rclcpp::executors::SingleThreadedExecutor>::ComponentManagerIsolated; // Uses the Constructor of ComponentManagerIsolated
    using stella_vslam_ros::System::System; // Uses the Constructor of System

    void Configure();

    void TimerCallback();

private:
    std::string mode_;
    std::shared_ptr<geo_calicam::MonocularCaliCam> calicam_;
    std::shared_ptr<Video> video_;
    std::shared_ptr<boost::circular_buffer<std::shared_ptr<stella_vslam::data::frame>>> buffer_;
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif // GEO_VSLAM__PRODUCER_HPP
