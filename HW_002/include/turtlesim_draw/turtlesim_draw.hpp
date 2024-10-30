#ifndef TURTLESIM_DRAW_HPP
#define TURTLESIM_DRAW_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/srv/set_pen.hpp"
#include <vector>
#include <string>

class TurtlesimDraw : public rclcpp::Node {
public:
    TurtlesimDraw();
    // 도형 그리는 함수
    void drawShape(const std::string &shape, double size);
    // 펜 색상과 굵기 설정 함수
    void setPenColor(int r, int g, int b, int width);

private:
    // 순서대로 직진, 회전, 삼각형, 원 그리는 함수
    void moveForward(double distance);
    void rotate(double angle);
    void drawTriangle(double side_length);
    void drawSquare(double side_length);
    void drawCircle(double radius);

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
    rclcpp::Client<turtlesim::srv::SetPen>::SharedPtr pen_client_;
};

#endif // TURTLESIM_DRAW_HPP

