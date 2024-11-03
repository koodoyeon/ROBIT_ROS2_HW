#include "../include/cam/main_window.hpp"

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent), ui(new Ui::MainWindowDesign) {
  ui->setupUi(this);
  
  QIcon icon("://ros-icon.png");
  this->setWindowIcon(icon);

  qnode = new QNode();

  QObject::connect(qnode, SIGNAL(rosShutDown()), this, SLOT(close()));
  QObject::connect(qnode, SIGNAL(imageReceived()), this, SLOT(updateImage()));  // 이미지 갱신 연결
}

void MainWindow::updateImage() {
  ui->camera_image_label->setPixmap(QPixmap::fromImage(qnode->getImage()));  // QLabel에 이미지 표시
}

void MainWindow::closeEvent(QCloseEvent* event) {
  QMainWindow::closeEvent(event);
}

MainWindow::~MainWindow() {
  delete ui;
}
