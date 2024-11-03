#ifndef cam_QNODE_HPP_
#define cam_QNODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <QImage>
#include <QThread>

class QNode : public QThread, public rclcpp::Node {
  Q_OBJECT
public:
  QNode();
  ~QNode();
  
  QImage getImage() const;  // 최신 이미지를 반환하는 함수

protected:
  void run();

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
  QImage current_image_;  // QLabel에 표시할 이미지를 저장할 변수
  
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);  // 이미지 콜백 함수

Q_SIGNALS:
  void rosShutDown();
  void imageReceived();  // 이미지가 갱신될 때 발생하는 시그널
};

#endif /* cam_QNODE_HPP_ */
