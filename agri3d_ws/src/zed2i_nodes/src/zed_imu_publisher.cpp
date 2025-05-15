#define SENSORS_MOD_AVAILABLE

#include <chrono>
#include <cmath>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensorcapture.hpp"
#include "sensorcapture_def.hpp"

class ZedImuPublisher : public rclcpp::Node
{
public:
    explicit ZedImuPublisher(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
        : Node("zed_imu_publisher", options),
        sens_(sl_oc::VERBOSITY::INFO)
    {
        this->declare_parameter<int>("serial_number", -1);
        this->declare_parameter<double>("pub_rate_hz", 100.0);
        this->declare_parameter<std::string>("frame_id", "zed_imu_link");

        serial_number_ = this->get_parameter("serial_number").as_int();
        auto pub_rate = this->get_parameter("pub_rate_hz").as_double();
        auto pub_period_ = std::chrono::duration<double>(1.0 / pub_rate);
        frame_id_ = this->get_parameter("frame_id").as_string();

        if (!sens_.initializeSensors(serial_number_)) {
            RCLCPP_FATAL(get_logger(), "Failed to initialize ZED 2I (serial %d)", serial_number_);
            throw std::runtime_error("ZED sensor init failed");
        }

        RCLCPP_INFO(get_logger(), "Connected to ZED (SN: %d)", sens_.getSerialNumber());

        publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", rclcpp::SensorDataQoS());
        timer_ = this->create_wall_timer(
                std::chrono::duration_cast<std::chrono::milliseconds>(pub_period_),
                std::bind(&ZedImuPublisher::publishImu, this));
    }

private:
    void publishImu()
    {
        const auto imu_data = sens_.getLastIMUData(1000);

        if (imu_data.valid != sl_oc::sensors::data::Imu::NEW_VAL) {
            RCLCPP_DEBUG(get_logger(), "No fresh IMU sample.");
            return;
        }

        sensor_msgs::msg::Imu msg;

        msg.header.stamp = rclcpp::Time(imu_data.timestamp, RCL_SYSTEM_TIME);
        msg.header.frame_id = frame_id_;

        msg.linear_acceleration.x = imu_data.aX;
        msg.linear_acceleration.y = imu_data.aY;
        msg.linear_acceleration.z = imu_data.aZ;

        constexpr double DEG2RAD = M_PI / 180.0;

        msg.angular_velocity.x = imu_data.gX * DEG2RAD;
        msg.angular_velocity.y = imu_data.gY * DEG2RAD;
        msg.angular_velocity.z = imu_data.gZ * DEG2RAD;
        
        msg.orientation_covariance[0] = -1.0;

        publisher_->publish(msg);
    }

    // init members
    
    sl_oc::sensors::SensorCapture sens_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::chrono::duration<double> pub_period_;
    std::string frame_id_;
    int serial_number_;
};

int main(int argc, char ** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ZedImuPublisher>());
    rclcpp::shutdown();
    return 0;
}
