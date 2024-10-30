#include "chatter_cli/listener.hpp"

Listener::Listener()
    : Node("listener") {
    string_subscription_ = this->create_subscription<std_msgs::msg::String>(
        "/chatter_cli", 10, std::bind(&Listener::chatterCallback, this, std::placeholders::_1));

    count_subscription_ = this->create_subscription<std_msgs::msg::Int64>(
        "/chatter_count", 10, std::bind(&Listener::countCallback, this, std::placeholders::_1));
}

// 문자열 메시지 처리 콜백 함수
void Listener::chatterCallback(const std_msgs::msg::String::SharedPtr msg) { 
// 구독한 문자열 메시지 로그에 출력
    RCLCPP_INFO(this->get_logger(), "Subscribed: '%s'", msg->data.c_str());
}

// 카운트 콜백 함수
void Listener::countCallback(const std_msgs::msg::Int64::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Count: %ld", msg->data);
}

// 메인
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Listener>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

