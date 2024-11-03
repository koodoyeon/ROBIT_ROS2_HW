#include <QApplication>
#include <rclcpp/rclcpp.hpp>
#include "../include/cam/main_window.hpp"

int main(int argc, char* argv[]) {
  // ROS2 초기화
  rclcpp::init(argc, argv);

  QApplication app(argc, argv);

  MainWindow window;
  window.show();

  int result = app.exec();

  // ROS2 종료
  rclcpp::shutdown();

  return result;
}
