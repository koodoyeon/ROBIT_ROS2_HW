#ifndef TALKER_HPP
#define TALKER_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int64.hpp"

class Talker : public rclcpp::Node {
public:
    Talker();
    void publishMessage(); 

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr string_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr count_publisher_;
    int64_t publish_count_;
};

#endif // TALKER_HPP

