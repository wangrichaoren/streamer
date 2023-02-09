#include <iostream>

#include <QtCore/QObject>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QMessageBox>
#include <opencv2/opencv.hpp>

#ifndef UTILS_HPP_
#define UTILS_HPP_

int createMessageBox(QWidget *parent,
                     const char *icon_file,
                     const char *window_title,
                     const char *text,
                     const int &btn_num,
                     const std::vector<const char *> &btn_texts);

QImage MatToQImage(const cv::Mat &mat);

QImage cvMat2QImage(const cv::Mat &mat);


#endif