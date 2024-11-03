#ifndef cam_MAIN_WINDOW_H
#define cam_MAIN_WINDOW_H

#include <QMainWindow>
#include "QIcon"
#include "qnode.hpp"
#include "ui_mainwindow.h"

class MainWindow : public QMainWindow {
  Q_OBJECT

public:
  MainWindow(QWidget* parent = nullptr);
  ~MainWindow();

public Q_SLOTS:
  void updateImage();  // QLabel을 업데이트하는 슬롯

private:
  Ui::MainWindowDesign* ui;
  QNode* qnode;
  void closeEvent(QCloseEvent* event);
};

#endif  // cam_MAIN_WINDOW_H
