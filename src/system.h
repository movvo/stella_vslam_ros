#ifdef HAVE_PANGOLIN_VIEWER
#include "pangolin_viewer/viewer.h"
#endif
#ifdef HAVE_SOCKET_PUBLISHER
#include "socket_publisher/publisher.h"
#endif

#include <stella_vslam/system.h>
#include <stella_vslam/config.h>
#include <stella_vslam/util/yaml.h>
#include <stella_vslam_ros.h>

#include <iostream>
#include <chrono>
#include <fstream>
#include <numeric>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <spdlog/spdlog.h>

#include <ghc/filesystem.hpp>
#include <boost/circular_buffer.hpp>

#include "geo_interfaces/msg/database.hpp"
#include "geo_interfaces/srv/transition.hpp"

using Mat44_t = Eigen::Matrix4d;

namespace stella_vslam_ros {

class System {
public:
    System(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    System(
        const std::string& name_space,
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    virtual ~System();

    std::shared_ptr<stella_vslam_ros::system> slam_ros_;
    std::shared_ptr<stella_vslam::system> slam_;
    std::shared_ptr<stella_vslam::config> cfg_;
    std::string map_db_path_out_;
    std::string viewer_string_;
    std::shared_ptr<std::thread> viewer_thread_;
#ifdef HAVE_PANGOLIN_VIEWER
    std::shared_ptr<pangolin_viewer::viewer> viewer_;
#endif
#ifdef HAVE_SOCKET_PUBLISHER
    std::shared_ptr<socket_publisher::publisher> publisher_;
#endif

    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
	get_node_base_interface() const;

    void TimerCallback();
    void LogWithTimestamp(std::string msg);
    void AddFrame(std::shared_ptr<stella_vslam::data::frame> & frame);
    void PublishResult(const std::shared_ptr<Mat44_t> & cam_pose_wc);
    void TransitionCallback(const std::shared_ptr<geo_interfaces::srv::Transition::Request> request,
                                std::shared_ptr<geo_interfaces::srv::Transition::Response> response);
    int id_;
    bool activated_{true};
    std::string map_db_path_;
    uint64_t last_timestamp_ = 0;
    rclcpp::Node::SharedPtr nh_;
    rclcpp::TimerBase::SharedPtr timer_;
    stella_vslam::tracker_state_t last_track_state_;
    rclcpp::Publisher<geo_interfaces::msg::Database>::SharedPtr status_pub_;
    rclcpp::Service<geo_interfaces::srv::Transition>::SharedPtr transition_srv_; /*< Service to trigger the transition of activation/deactivation */
    std::mutex buffer_mutex_;
    boost::circular_buffer<std::shared_ptr<stella_vslam::data::frame>> buffer_;
};

} // namespace stella_vslam_ros
