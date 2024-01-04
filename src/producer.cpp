#include <chrono>
#include <producer.hpp>

using namespace std::chrono_literals;

namespace {
    void LogWithTimestamp(std::string msg) {
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        std::cout<<"["<<ms<<"][Producer]: " << msg << std::endl;
    }
}

Video::Video(std::string path_to_video) : cap_(path_to_video)
{
    if(!cap_.isOpened()) {
        ::LogWithTimestamp("Error opening video file");
    }
}

Video::~Video()
{
    // When everything done, release the video capture object
    cap_.release();
}

bool Video::GrabFrame(cv::Mat & frame)
{
    // Capture frame-by-frame
    cap_ >> frame;
    
    // If the frame is empty, break immediately
    if (frame.empty()) {
        return false;
    }
    return true;
}

// Component Manager a.k.a Producer
void Producer::Configure()
{
    double frequency = declare_parameter("timer_frequency", 30.0);
    timer_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    timer_ = create_wall_timer(std::chrono::duration<double>(1/frequency), std::bind(&Producer::TimerCallback, this), timer_callback_group_);

    mode_ = declare_parameter("mode", "video");
    if (mode_ == "camera") {
        // Camera
        camera_ = std::make_shared<geo_camera::MonocularCalicam>("Calicam", shared_from_this());
    }
    else if (mode_ == "video") {
        // Video
        std::string video_source = declare_parameter("video_source", "/inputs/aist_living_lab_3/video.mp4");
        video_ = std::make_shared<Video>(video_source);
    }
    else {
        RCLCPP_ERROR(get_logger(), "Unknown type of image source");
    }
}

void Producer::TimerCallback()
{
    cv::Mat frame, mask;
    if (mode_ == "camera") {
        if (!camera_->GrabFrame(frame)) {
            return;
        }   
    }
    else if (mode_ == "video") {
        if (!video_->GrabFrame(frame)) {
            return;
        }
    }
    
    const double timestamp = now().seconds();

    // ::LogWithTimestamp("Start creating monocular frame");
    auto data_frame = slam_->create_monocular_frame(1, frame, timestamp, mask);
    auto data_frame_ptr = std::make_shared<stella_vslam::data::frame>(data_frame);
    // ::LogWithTimestamp("Finish creating monocular frame with memaddres 0x" + std::to_string(reinterpret_cast<std::uintptr_t>(data_frame_ptr.get())));
    // ::LogWithTimestamp("Finish creating monocular frame with id " + std::to_string(data_frame_ptr->id_));

    // uint64_t timestamp_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    // if (last_timestamp_ != 0) {
    //     std::cout << "[Producer-" << id_ << "]: (frame id" << data_frame_ptr->id_ << ") Time between frames in ms: " << timestamp_ms - last_timestamp_ << std::endl;
    // }
    // last_timestamp_ = timestamp_ms;

    // Feed to components
    for (auto & [id, node] : node_wrappers_) {
        auto vslam_component = std::static_pointer_cast<stella_vslam_ros::System>(node.get_node_instance());
        vslam_component->AddFrame(data_frame_ptr);
    }
}