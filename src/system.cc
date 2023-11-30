#include <system.h>
#include <stella_vslam/data/frame.h>

namespace fs = ghc::filesystem;

using namespace std::chrono_literals;

namespace stella_vslam_ros {

const static int BUFFER_LENGTH = 100;

System::System(
    const rclcpp::NodeOptions& options)
    : System("", options) {}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
System::get_node_base_interface() const
{
    return nh_->get_node_base_interface();
}

System::System(
    const std::string& name_space,
    const rclcpp::NodeOptions& options)
    : nh_(std::make_shared<rclcpp::Node>("run_slam", name_space, options)) {
    std::string vocab_file_path = nh_->declare_parameter("vocab_file_path", "");
    std::string setting_file_path = nh_->declare_parameter("setting_file_path", "");
    std::string log_level = nh_->declare_parameter("log_level", "info");
    std::string map_db_path_in = nh_->declare_parameter("map_db_path_in", "");
    map_db_path_out_ = nh_->declare_parameter("map_db_path_out", "");
    bool disable_mapping = nh_->declare_parameter("disable_mapping", true);
    bool temporal_mapping = nh_->declare_parameter("temporal_mapping", false);
    std::string viewer = nh_->declare_parameter("viewer", "none");

    if (vocab_file_path.empty() || setting_file_path.empty()) {
        RCLCPP_FATAL(nh_->get_logger(), "Invalid parameter");
        return;
    }

    // viewer
    if (!viewer.empty()) {
        viewer_string_ = viewer;
        if (viewer_string_ != "pangolin_viewer" && viewer_string_ != "socket_publisher" && viewer_string_ != "none") {
            RCLCPP_FATAL(nh_->get_logger(), "invalid arguments (--viewer)");
            return;
        }
#ifndef HAVE_PANGOLIN_VIEWER
        if (viewer_string_ == "pangolin_viewer") {
            RCLCPP_FATAL(nh_->get_logger(), "pangolin_viewer not linked");
            return;
        }
#endif
#ifndef HAVE_SOCKET_PUBLISHER
        if (viewer_string_ == "socket_publisher") {
            RCLCPP_FATAL(nh_->get_logger(), "socket_publisher not linked");
            return;
        }
#endif
    }
    else {
#ifdef HAVE_PANGOLIN_VIEWER
        viewer_string_ = "pangolin_viewer";
#elif defined(HAVE_SOCKET_PUBLISHER)
        viewer_string_ = "socket_publisher";
#endif
    }

    // setup logger
    spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] %^[%L] %v%$");
    spdlog::set_level(spdlog::level::from_str(log_level));

    // load configuration
    try {
        cfg_ = std::make_shared<stella_vslam::config>(setting_file_path);
    }
    catch (const std::exception& e) {
        RCLCPP_FATAL(nh_->get_logger(), e.what());
        return;
    }

    slam_ = std::make_shared<stella_vslam::system>(cfg_, vocab_file_path);
    bool need_initialize = false;
    if (!map_db_path_in.empty()) {
        need_initialize = false;
        const auto path = fs::path(map_db_path_in);
        if (path.extension() == ".yaml") {
            YAML::Node node = YAML::LoadFile(path);
            for (const auto& map_path : node["maps"].as<std::vector<std::string>>()) {
                slam_->load_map_database(path.parent_path() / map_path);
            }
        }
        else {
            // load the prebuilt map
            slam_->load_map_database(path);
        }
    }
    slam_->startup(need_initialize);
    if (disable_mapping) {
        slam_->disable_mapping_module();
        slam_->disable_loop_detector();
    }
    else if (temporal_mapping) {
        slam_->enable_temporal_mapping();
        slam_->disable_loop_detector();
    }

    if (slam_->get_camera()->setup_type_ == stella_vslam::camera::setup_type_t::Monocular) {
        slam_ros_ = std::make_shared<stella_vslam_ros::mono>(slam_, nh_.get(), "");
    }
    else {
        RCLCPP_FATAL_STREAM(nh_->get_logger(), "Invalid setup type: " << slam_->get_camera()->get_setup_type_string());
        return;
    }

    // create a viewer object
    // and pass the frame_publisher and the map_publisher
#ifdef HAVE_PANGOLIN_VIEWER
    if (viewer_string_ == "pangolin_viewer") {
        viewer_ = std::make_shared<pangolin_viewer::viewer>(
            stella_vslam::util::yaml_optional_ref(cfg_->yaml_node_, "PangolinViewer"),
            slam_,
            slam_->get_frame_publisher(),
            slam_->get_map_publisher());
    }
#endif
#ifdef HAVE_SOCKET_PUBLISHER
    if (viewer_string_ == "socket_publisher") {
        publisher_ = std::make_shared<socket_publisher::publisher>(
            stella_vslam::util::yaml_optional_ref(cfg_->yaml_node_, "SocketPublisher"),
            slam_,
            slam_->get_frame_publisher(),
            slam_->get_map_publisher());
    }
#endif

    if (viewer_string_ != "none") {
        // TODO: Pangolin needs to run in the main thread on OSX
        // run the viewer in another thread
        viewer_thread_ = std::make_shared<std::thread>([&, this]() {
            if (viewer_string_ == "pangolin_viewer") {
#ifdef HAVE_PANGOLIN_VIEWER
                viewer_->run();
#endif
            }
            if (viewer_string_ == "socket_publisher") {
#ifdef HAVE_SOCKET_PUBLISHER
                publisher_->run();
#endif
            }
            if (slam_->terminate_is_requested()) {
                // wait until the loop BA is finished
                while (slam_->loop_BA_is_running()) {
                    std::this_thread::sleep_for(std::chrono::microseconds(5000));
                }
                rclcpp::shutdown();
            }
        });
    }

    timer_ = nh_->create_wall_timer(33ms, std::bind(&System::TimerCallback, this));
    buffer_ = boost::circular_buffer<std::shared_ptr<stella_vslam::data::frame>>(BUFFER_LENGTH);
    id_ = rand();
}

System::~System() {
    // automatically close the viewer
    if (viewer_string_ == "pangolin_viewer") {
#ifdef HAVE_PANGOLIN_VIEWER
        viewer_->request_terminate();
#endif
    }
    if (viewer_string_ == "socket_publisher") {
#ifdef HAVE_SOCKET_PUBLISHER
        publisher_->request_terminate();
#endif
    }
    if (viewer_string_ != "none") {
        viewer_thread_->join();
    }

    // shutdown the SLAM process
    slam_->shutdown();

    if (!map_db_path_out_.empty()) {
        // output the map database
        slam_->save_map_database(map_db_path_out_);
    }
}

void System::LogWithTimestamp(std::string msg)
{
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    std::cout<<"["<<ms<<"][Consumer-" << id_ << "]: " << msg << std::endl;
}

void System::TimerCallback()
{
    std::shared_ptr<stella_vslam::data::frame> data_frame_ptr;
    {
        std::lock_guard<std::mutex> lock(buffer_mutex_);
        if (!buffer_.empty()) {
            data_frame_ptr = buffer_.front();
            buffer_.pop_front();
        }
    }
    
    if (data_frame_ptr) {
        cv::Mat img; // Empty image because is for frame_publisher
        // LogWithTimestamp("Feeding frame with memaddres: 0x" + std::to_string(reinterpret_cast<std::uintptr_t>(data_frame_ptr.get())));
        LogWithTimestamp("Feeding frame with id " + std::to_string(data_frame_ptr->id_));
        auto res = slam_->feed_frame(id_, *data_frame_ptr.get(), img);
        LogWithTimestamp("Ended feeding frame");
    }
}

void System::AddFrame(std::shared_ptr<stella_vslam::data::frame> & frame)
{
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    buffer_.push_back(frame);
}

} // namespace stella_vslam_ros

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(stella_vslam_ros::System)
