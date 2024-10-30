#include "turtlesim_draw/turtlesim_draw.hpp"
#include <chrono>
#include <cmath>
#include <iostream>

TurtlesimDraw::TurtlesimDraw() : Node("turtlesim_draw") {
    velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
    pen_client_ = this->create_client<turtlesim::srv::SetPen>("/turtle1/set_pen");
}

// 펜 색상, 굵기
void TurtlesimDraw::setPenColor(int r, int g, int b, int width) {
    auto request = std::make_shared<turtlesim::srv::SetPen::Request>();
    request->r = r;  // 펜 색상 R,G,B
    request->g = g;
    request->b = b;
    request->width = width; // 펜 굵기

    while (!pen_client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_WARN(this->get_logger(), "Waiting for the pen service to be available...");
    }
    
    // 색상과 굵기 설정하는 서비스 요청
    auto future = pen_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) != 
        rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(this->get_logger(), "Failed to call service set_pen");
    }
}

// 직진
void TurtlesimDraw::moveForward(double distance) {
    geometry_msgs::msg::Twist twist;
    twist.linear.x = distance > 0 ? 1.0 : -1.0; 
    twist.angular.z = 0.0; // 회전 속도 0.

    double duration = std::abs(distance);  // 이동 거리만큼 시간 설정
    auto start_time = this->now(); // 시작 시간 기록

    while ((this->now() - start_time).seconds() < duration) { // 지정 시간 만큼 이동
        velocity_publisher_->publish(twist); // 속도
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

    // 정지
    twist.linear.x = 0.0; // 속도 0으로 설정
    velocity_publisher_->publish(twist); 
}

// 회전
void TurtlesimDraw::rotate(double angle) {
    geometry_msgs::msg::Twist twist;
    twist.linear.x = 0.0; // 직진 속도 0
    twist.angular.z = angle > 0 ? 1.0 : -1.0; 

    double duration = std::abs(angle);
    auto start_time = this->now();

// 지정 시간 만큼 회전
    while ((this->now() - start_time).seconds() < duration) {
        velocity_publisher_->publish(twist);
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

    twist.angular.z = 0.0;
    velocity_publisher_->publish(twist);
}


void TurtlesimDraw::drawShape(const std::string &shape, double size) {
    if (shape == "triangle") {
        drawTriangle(size); // 삼각형 그리기
    } else if (shape == "square") {
        drawSquare(size); // 사각형 그리기
    } else if (shape == "circle") {
        drawCircle(size); // 원 그리기
    } else {
        RCLCPP_WARN(this->get_logger(), "Unknown shape: %s", shape.c_str());
    }
}

// 삼각형
void TurtlesimDraw::drawTriangle(double side_length) {
    for (int i = 0; i < 3; ++i) {
        moveForward(side_length); // 한 변 그림
        rotate(2 * M_PI / 3);  // 120도 회전
    }
}

// 사각형
void TurtlesimDraw::drawSquare(double side_length) {
    for (int i = 0; i < 4; ++i) {
        moveForward(side_length); // 한 변 그림
        rotate(M_PI / 2);  // 90도 회전
    }
}

// 원
void TurtlesimDraw::drawCircle(double radius) {
    geometry_msgs::msg::Twist twist;
    twist.linear.x = 1.0;
    twist.angular.z = 1.0 / radius; // 원의 반지름 따라 회전 속도 지정 

    auto start_time = this->now();
    double duration = 2 * M_PI * radius;  // 원 둘레

    while ((this->now() - start_time).seconds() < duration) {
        velocity_publisher_->publish(twist);
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

    // 정지
    twist.linear.x = 0.0;
    twist.angular.z = 0.0;
    velocity_publisher_->publish(twist);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtlesimDraw>();

    // 펜 색상, 굵기 입력 받기
    int r, g, b, width;
    std::cout << "펜의 색상(R G B)과 굵기를 입력하세요 (예: 255 0 0 3): ";
    std::cin >> r >> g >> b >> width; // 색상, 굵기 입력 받음
    node->setPenColor(r, g, b, width);

    // 도형 선택
    std::string shape;
    double size;
    std::cout << "그릴 도형을 선택하세요 (triangle, square, circle): ";
    std::cin >> shape; // 그릴 도형 입력 받기
    std::cout << "도형의 크기를 입력하세요: ";
    std::cin >> size; // 크기 입력 받기

    // 그리기
    node->drawShape(shape, size);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

