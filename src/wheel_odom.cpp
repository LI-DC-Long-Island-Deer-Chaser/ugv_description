#include <gpiod.hpp>
#include <functional>
#include <memory>
#include <string>
#include <cmath>
#include <iostream>
#include <thread>
#include <atomic>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"

using namespace std;

int LINE_A = 106;  // GPIO11 (Pin 31)
int LINE_B = 43;   // GPIO13 (Pin 33)

string CHIP = "/dev/gpiochip0";


class WheelEncoder : public rclcpp::Node
{
    public:
        using Header = std_msgs::msg::Header;
        using Twist = geometry_msgs::msg::Twist;
        using Vector3 = geometry_msgs::msg::Vector3;
        using TwistWCovS = geometry_msgs::msg::TwistWithCovarianceStamped;

        WheelEncoder() : Node("wheel_encoder")
        {
            auto param_desc_ctmcf = rcl_interfaces::msg::ParameterDescriptor{};
            param_desc_ctmcf.description = "This parameter is used as a fixed calibration value derived experimentally to match the rovers counts to meters ratio.";
            this->declare_parameter("count_to_meter_conversion_factor", 110.021e-05, param_desc_ctmcf);
            auto param_desc_stddev = rcl_interfaces::msg::ParameterDescriptor{};
            param_desc_stddev.description = "This parameter is used as a fixed calibration value derived experimentally to match the rovers counts to meters model uncertainty.";
            this->declare_parameter("standard_deviation", 4.023e-05, param_desc_stddev);
            auto param_desc_yslip = rcl_interfaces::msg::ParameterDescriptor{};
            param_desc_yslip.description = "This parameter is used as a confidence value for non-slipage in the y dirction. Some slipage is unavoidable so a value of 0.1 to 0.3 is recommened by chat.";
            this->declare_parameter("y_slip_factor", 0.1, param_desc_yslip);
            

            previous_time_ = this->get_clock()->now();

            chip_ = CHIP;
            position_ = 0;
            dir_cw_ = true;
            previous_position_ = position_.load();

            lineA_ = chip_.get_line(LINE_A);
            configA_.consumer = "quadA";
            configA_.request_type = gpiod::line_request::EVENT_BOTH_EDGES;
            lineA_.request(configA_);

            lineB_ = chip_.get_line(LINE_B);
            configB_.consumer = "quadB";
            configB_.request_type = gpiod::line_request::EVENT_BOTH_EDGES;
            lineB_.request(configB_);

            thread([this]()
            {
                decodeA(lineA_, lineB_);
            }).detach();

            thread([this]()
            {
                decodeB(lineA_, lineB_);                
            }).detach();
            
            this->publisher_ = this->create_publisher<TwistWCovS>("/ugv/wheel_odom", rclcpp::SensorDataQoS());
            this->timer_ = this->create_wall_timer(chrono::milliseconds(10), bind(&WheelEncoder::publish_pose, this));
        }

    private:
        atomic<size_t> position_;
        atomic<bool> dir_cw_;

        gpiod::chip chip_;

        gpiod::line lineA_ ;
        gpiod::line lineB_;

        gpiod::line_request configA_;
        gpiod::line_request configB_;

        rclcpp::Publisher<TwistWCovS>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;

        size_t previous_position_;
        rclcpp::Time previous_time_;

        void decodeA(gpiod::line& lineA, gpiod::line& lineB)
        {
            while (true)
            {
                lineA.event_wait(chrono::nanoseconds(1000));
                auto evt = lineA.event_read();

                bool a = lineA.get_value();
                bool b = lineB.get_value();

                if (a == b)
                {
                    this->position_++;
                    this->dir_cw_ = true;
                }
                else
                {
                    this->position_--;
                    this->dir_cw_ = false;
                }
            }
        }

        void decodeB(gpiod::line& lineA, gpiod::line& lineB)
        {
            while (true)
            {
                lineB.event_wait(chrono::nanoseconds(1000));
                auto evt = lineB.event_read();

                bool a = lineA.get_value();
                bool b = lineB.get_value();

                if (a != b)
                {
                    this->position_++;
                    this->dir_cw_ = true;
                } else
                {
                    this->position_--;
                    this->dir_cw_ = false;
                }
            }
        }

        void publish_pose()
        {
            long double count_to_meter = this->get_parameter("count_to_meter_conversion_factor").as_double();
            long double std = this->get_parameter("standard_deviation").as_double();
            long double y_slip_alpha = this->get_parameter("y_slip_factor").as_double() + 1;
            long double dt = 0;
            long double dx = 0;

            auto head = Header();
            auto twist = Twist();
            auto angular = Vector3();
            auto linear = Vector3();
            auto twcovs = TwistWCovS();

            size_t p = this->position_.load();
            bool d = this->dir_cw_.load();

            head.frame_id = "base_link";
            head.stamp = this->get_clock()->now();

            dt =  (double)(head.stamp.sec - previous_time_.seconds()) + (double)(head.stamp.nanosec - previous_time_.nanoseconds()) * 1.0e-9;
            dx = (p - previous_position_) * count_to_meter;

            twcovs.header = head;

            linear.x = (dx / dt);
            linear.y = 0.0;
            linear.z = 0.0;
            twist.linear = linear;

            previous_time_ = head.stamp;
            previous_position_ = p;

            angular.x = 0.0;
            angular.y = 0.0;
            angular.z = 0.0;
            twist.angular = angular;

            twcovs.twist.twist = twist;

            fill(twcovs.twist.covariance.begin(), twcovs.twist.covariance.end(), 1e3);
            twcovs.twist.covariance[0] = pow(std, 2) * pow(linear.x / count_to_meter, 2);
            twcovs.twist.covariance[7] = pow(std * y_slip_alpha, 2) * pow(linear.x / count_to_meter, 2);
            twcovs.twist.covariance[1] = std * std * y_slip_alpha * pow(linear.x / count_to_meter, 2);
            twcovs.twist.covariance[6] = std * std * y_slip_alpha * pow(linear.x / count_to_meter, 2);

            publisher_->publish(twcovs);
            RCLCPP_INFO(this->get_logger(), "position: %ld, direction: %s", p, (d ? "CW" : "CCW"));
        }
};

int main(int argc, char * argv[]) 
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WheelEncoder>());
    rclcpp::shutdown();
    return 0;
}
