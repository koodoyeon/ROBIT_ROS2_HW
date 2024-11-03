#include "../include/cam/qnode.hpp"

QNode::QNode() : QThread(), Node("cam_node") {
  // ROS2 초기화는 main.cpp에서 수행하므로 여기서는 호출하지 않습니다.
  
  // 카메라 이미지 구독자를 /camera1/camera/image_raw 토픽에 맞게 초기화
  image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/camera1/camera/image_raw", 10, std::bind(&QNode::image_callback, this, std::placeholders::_1)
  );

  this->start();
}

QNode::~QNode() {
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
}

void QNode::run() {
  rclcpp::spin(this->get_node_base_interface());  // ROS 이벤트 루프 실행
  Q_EMIT rosShutDown();
}

// 이미지 콜백 함수
void QNode::image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
  try {
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat mat = cv_ptr->image;

    // OpenCV 이미지를 QImage로 변환하고, QLabel 크기에 맞게 조정
    current_image_ = QImage(mat.data, mat.cols, mat.rows, mat.step, QImage::Format_BGR888)
                     .scaled(640, 480, Qt::KeepAspectRatio);

    Q_EMIT imageReceived();  // 이미지를 받았음을 알림
  } catch (cv_bridge::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge 예외 발생: %s", e.what());  // 예외 발생 시 로깅
  }
}

QImage QNode::getImage() const {
  return current_image_;
}
