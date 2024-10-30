#include "chatter_cli/talker.hpp"
#include <iostream>

Talker::Talker()
    : Node("talker"), publish_count_(0) {
    string_publisher_ = this->create_publisher<std_msgs::msg::String>("/chatter_cli", 10);
    count_publisher_ = this->create_publisher<std_msgs::msg::Int64>("/chatter_count", 10);

    RCLCPP_INFO(this->get_logger(), "Talker node has started.");
}

void Talker::publishMessage() {
    while (rclcpp::ok()) {
        // 사용자 입력 대기
        std::cout << "publish: ";
        std::string user_input;
        std::getline(std::cin, user_input);

        // 입력이 빈 문자열인 경우 종료
        if (user_input.empty()) {
            RCLCPP_INFO(this->get_logger(), "No message entered. Exiting...");
            break;
        }

        // 문자열 메시지 생성 및 퍼블리시
        auto message = std_msgs::msg::String();
        message.data = user_input;
        string_publisher_->publish(message);

        // 카운트 메시지 생성 및 퍼블리시
        auto count_message = std_msgs::msg::Int64();
        count_message.data = ++publish_count_;
        count_publisher_->publish(count_message);

        RCLCPP_INFO(this->get_logger(), "published message: '%s' count: %ld", user_input.c_str(), publish_count_);
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Talker>();
    
    // 사용자 입력에 따른 퍼블리시를 처리
    node->publishMessage();
    
    rclcpp::shutdown();
    return 0;
}

