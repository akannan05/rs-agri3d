#define VIDEO_MOD_AVAILABLE

#include <memory>
#include <chrono>
#include <stdexcept>
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.hpp>

#include "videocapture.hpp"
#include "videocapture_def.hpp"

class ZedStereoPublisher : public rclcpp::Node {
public:
    ZedStereoPublisher() 
        : Node("zed_stereo_publisher"),
        cap_(initVideoParams())
    {
        if (!cap_.initializeVideo()) {
            RCLCPP_ERROR(get_logger(), "Failed to initialize ZED video capture.");
            rclcpp::shutdown();
            return;
        }

        left_pub_ = image_transport::create_publisher(this, "left/image_raw");
        right_pub_ = image_transport::create_publisher(this, "right/image_raw");

        timer_ = create_wall_timer(
                std::chrono::milliseconds(33), 
                std::bind(&ZedStereoPublisher::publish_frames, this));
    }

private:
    void publish_frames() {
        const sl_oc::video::Frame frame = cap_.getLastFrame();
        if (frame.data != nullptr) {
            cv::Mat stereo_img(frame.height, frame.width, CV_8UC2, frame.data);

            cv::Mat left_yuyv = stereo_img.colRange(0, frame.width / 2);
            cv::Mat right_yuyv = stereo_img.colRange(frame.width / 2, frame.width);

            cv::Mat left_bgr, right_bgr;
            cv::cvtColor(left_yuyv, left_bgr, cv::COLOR_YUV2BGR_YUYV);
            cv::cvtColor(right_yuyv, right_bgr, cv::COLOR_YUV2BGR_YUYV);

            auto timestamp = this->get_clock()->now();

            auto left_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", left_bgr).toImageMsg();
            left_msg->header.stamp = timestamp;
            left_msg->header.frame_id = "zed_left";

            auto right_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", right_bgr).toImageMsg();
            right_msg->header.stamp = timestamp;
            right_msg->header.frame_id = "zed_right";

            left_pub_.publish(left_msg);
            right_pub_.publish(right_msg);
        }
    }
    static sl_oc::video::VideoParams initVideoParams() {
        sl_oc::video::VideoParams params;
        params.res = sl_oc::video::RESOLUTION::HD720;
        params.fps = sl_oc::video::FPS::FPS_30;
        params.verbose = sl_oc::VERBOSITY::INFO; 
        return params;
    }

    sl_oc::video::VideoCapture cap_;
    image_transport::Publisher left_pub_, right_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ZedStereoPublisher>());
    rclcpp::shutdown();
    return 0;
}
