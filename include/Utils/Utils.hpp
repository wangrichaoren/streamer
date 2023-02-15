#include <iostream>

#include <opencv2/opencv.hpp>
#include <QtCore/QObject>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMessageBox>
#include <QtWidgets/QVBoxLayout>

#ifndef UTILS_HPP_
#define UTILS_HPP_

int createMessageBox(QWidget *parent,
                     const char *icon_file,
                     const char *window_title,
                     const char *text,
                     const int &btn_num,
                     const std::vector<const char *> &btn_texts);

void createLineEditFormCurQObj(QHBoxLayout *hlay_obj, QLabel *lab_obj, QLineEdit *edit_obj);

QImage MatToQImage(const cv::Mat &mat);

QImage cvMat2QImage(const cv::Mat &mat);

bool getDirectoryFile(const std::string &dir_in, std::vector<std::string> &files);

#endif